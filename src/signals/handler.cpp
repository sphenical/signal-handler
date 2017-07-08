/*
 * LICENSE
 *
 * Copyright (c) 2017, David Daniel (dd), david@daniels.li
 *
 * handler.cpp is free software copyrighted by David Daniel.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * This program comes with ABSOLUTELY NO WARRANTY.
 * This is free software, and you are welcome to redistribute it
 * under certain conditions.
 */
#include    <signals/handler.hpp>

#include    <cstdint>
#include    <limits>
#include    <cstring>
#include    <vector>
#include    <map>
#include    <stdexcept>
#include    <chrono>
#include    <thread>
#include    <atomic>

#include    <unistd.h>
#include    <sys/select.h>
#include    <signal.h>
#include    <fcntl.h>

namespace signals {

    namespace {
        static constexpr const std::size_t MaxErrors = 5;

        class ReadError : public std::runtime_error
        {
            public:
                using std::runtime_error::runtime_error;
        };

        void catch_signal (int);

        static constexpr const std::uint8_t ShutdownSignal
            = std::numeric_limits<std::uint8_t>::max ();

        struct Dispatcher
        {
            struct sigaction ancestor;
            const Handler* handler;

            Dispatcher (const Handler*);
        };

        Dispatcher::Dispatcher (const Handler* h) :
            ancestor {},
            handler {h}
        {}

        class Registry
        {
            using Dispatchers = std::vector<Dispatcher>;
            using Handlers = std::map<int, Dispatchers>;
            using Callables = std::map<const Handler*, Handler::Sink>;

            public:
                static Registry& instance ();
                ~Registry ();

                template <typename Func>
                    Registry& addHandler (const Handler*, Func&&);

                Registry& removeHandler (const Handler*);

                Registry& push (int, const Handler*);
                Registry& pop (int, const Handler*);

                void handle (int);

            private:
                Registry ();

                void readSignals ();

                void closePipe ();

            private:
                static Registry instance_;

            private:
                Handlers handlers_;
                Callables callables_;
                int readFd_;
                int writeFd_;
                std::thread thread_;
                std::atomic_bool running_;
        };

        Registry Registry::instance_ {};

        inline Registry& Registry::instance ()
        { return instance_; }

        Registry::Registry () :
            handlers_ {},
            callables_ {},
            readFd_ {},
            writeFd_ {},
            thread_ {},
            running_ {true}
        {
            int fds [2];
            if (::pipe (fds) != 0) {
                throw std::runtime_error {
                    "Cannot create a pipe for communicating signals."};
            }

            readFd_ = fds [0];
            writeFd_ = fds [1];

            if (fcntl (writeFd_, F_SETFL, fcntl (writeFd_, F_GETFL) | O_NONBLOCK) < 0) {
                closePipe ();
                throw std::runtime_error {
                    "Cannot set the write filedescriptor of the signal pipe to non-blocking."};
            }

            if (fcntl (writeFd_, F_SETFD, fcntl (writeFd_, F_GETFD) | FD_CLOEXEC) < 0) {
                closePipe ();
                throw std::runtime_error {
                    "Cannot set the write filedescriptor of the signal pipe to close on exec."};
            }

            if (fcntl (readFd_, F_SETFD, fcntl (readFd_, F_GETFD) | FD_CLOEXEC) < 0) {
                closePipe ();
                throw std::runtime_error {
                    "Cannot set the read filedescriptor of the signal pipe to close on exec."};
            }

            thread_ = std::thread {&Registry::readSignals, this};
        }

        Registry::~Registry ()
        {
            running_.exchange (false);
            std::size_t errorCounter = 0;

            while (errorCounter < MaxErrors
                    && ::write (writeFd_, &ShutdownSignal, sizeof ShutdownSignal) != sizeof ShutdownSignal)
            {
                ++errorCounter;
            }

            thread_.join ();
            closePipe ();
        }

        template <typename Func>
            inline Registry& Registry::addHandler (const Handler* handler, Func&& func)
            {
                callables_.emplace (handler, std::forward<Func> (func));
                return *this;
            }

        inline Registry& Registry::removeHandler (const Handler* handler)
        {
            callables_.erase (handler);
            return *this;
        }

        Registry& Registry::push (int signum, const Handler* handler)
        {
            struct sigaction action;

            std::memset (&action, 0, sizeof action);

            action.sa_handler = &catch_signal;
            action.sa_flags = 0;
            sigfillset (&action.sa_mask);

            Dispatcher dispatcher {handler};

            if (::sigaction (signum, &action, &dispatcher.ancestor) < 0) {
                throw std::runtime_error {
                    "Cannot register a signal handler for the given signal."};
            }

            Handlers::iterator atSignal = handlers_.find (signum);
            if (atSignal == handlers_.end ()) {
                atSignal = handlers_.emplace (signum, Dispatchers {})
                    .first;
            }

            Dispatchers& dispatchers = atSignal->second;
            dispatchers.push_back (std::move (dispatcher));

            return *this;
        }

        void Registry::handle (int signum)
        {
            std::uint8_t reduced = static_cast<std::uint8_t> (signum);
            std::size_t errorCounter = 0;
            while (errorCounter < MaxErrors
                    && ::write (writeFd_, &reduced, sizeof reduced) != sizeof reduced)
            {
                ++errorCounter;
            }
        }

        Registry& Registry::pop (int signum, const Handler* handler)
        {
            Handlers::iterator atHandler = handlers_.find (signum);
            if (atHandler != handlers_.end ()) {
                Dispatchers& dispatchers = atHandler->second;
                Dispatchers::reverse_iterator atDispatcher = dispatchers.rbegin ();
                Dispatchers::reverse_iterator endOfDispatchers = dispatchers.rend ();

                bool onTop = true;
                while (onTop && atDispatcher != endOfDispatchers) {
                    if (atDispatcher->handler == handler) {

                        ::sigaction (signum, &atDispatcher->ancestor, nullptr);

                        atDispatcher = Dispatchers::reverse_iterator {
                            dispatchers.erase (
                                    std::next (atDispatcher).base ())};
                    }
                    else {
                        onTop = false;
                        ++atDispatcher;
                    }
                }
                while (atDispatcher != endOfDispatchers) {
                    if (atDispatcher->handler == handler) {

                        auto previous = std::prev (atDispatcher);
                        previous->ancestor = std::move (atDispatcher->ancestor);

                        atDispatcher = Dispatchers::reverse_iterator {
                            dispatchers.erase (
                                    std::next (atDispatcher).base ())};
                    }
                    else {
                        ++atDispatcher;
                    }
                }

                if (atHandler->second.empty ()) {
                    handlers_.erase (atHandler);
                }
            }

            return *this;
        }

        void Registry::readSignals ()
        {
            std::size_t readErrors = 0;

            while (readErrors < MaxErrors && running_.load ()) {
                fd_set fds;
                FD_ZERO (&fds);
                FD_SET (readFd_, &fds);

                timeval timeout {
                    .tv_sec = 60 * 60,
                    .tv_usec = 0
                };

                try {
                    int nrFds = ::select (readFd_ + 1, &fds, nullptr, nullptr, &timeout);
                    if (nrFds < 0) {
                        throw ReadError {
                            "Cannot select on the signal pipes read file descriptor."};
                    }
                    else if (FD_ISSET (readFd_, &fds)) {
                        std::uint8_t signum;
                        ssize_t bytes = ::read (readFd_, &signum, sizeof signum);

                        if (bytes != sizeof signum) {
                            throw ReadError {"Cannot read from the signal pipe."};
                        }
                        else if (signum != ShutdownSignal) {
                            readErrors = 0;
                            Handlers::iterator atHandler = handlers_.find (signum);

                            if (atHandler != handlers_.end ()) {
                                Dispatchers& dispatchers = atHandler->second;

                                if (!dispatchers.empty ()) {
                                    const Handler* handler = dispatchers.back ().handler;

                                    Callables::const_iterator atCallable = callables_.find (handler);
                                    if (atCallable != callables_.cend () && atCallable->second) {
                                        (atCallable->second) (signum);
                                    }
                                }
                            }
                        }
                    }
                }
                catch (const ReadError&) {
                    ++readErrors;
                    std::this_thread::sleep_for (std::chrono::milliseconds {20});
                }
                catch (...) {
                    // "The execution of the signal handler raised an exception."
                }
            }
        }

        inline void Registry::closePipe ()
        {
            ::close (readFd_);
            ::close (writeFd_);
        }

        void catch_signal (int signum)
        {
            Registry::instance ().handle (signum);
        }
    }

    /**
     * Unregisters the handler from the internal registry.
     */
    Handler::~Handler ()
    {
        Registry& registry = Registry::instance ();

        for (const auto sig : signals_) {
            registry.pop (sig, this);
        }

        registry.removeHandler (this);
    }

    /**
     * Registers the handler in the internal registry.
     */
    void Handler::start (Sink&& callback)
    {
        Registry::instance ()
            .addHandler (this, std::move (callback));
    }

    /**
     * Adds the given signal to the set of signals listened to.
     * If the signal is already listened to, this function does nothing.
     * After this function returns the given signal will be delivered to the stored
     * callback.
     * @param signum the signal to additionally listen on
     * @return this handler
     */
    Handler& Handler::addSignal (int signum)
    {
        Signals::const_iterator atSignal = signals_.find (signum);

        if (atSignal == signals_.cend ()) {
            Registry::instance ().push (signum, this);
            signals_.insert (signum);
        }

        return *this;
    }

    /**
     * Removes the given signal from the set of signals listened to.
     * If the signal is not listened to, this function does nothing.
     * After this function returns the given signal will not be delivered to the stored
     * callback any more.
     * @param signum the signal to remove from the set of signals listened to
     * @return this handler
     */
    Handler& Handler::removeSignal (int signum)
    {
        Signals::const_iterator atSignal = signals_.find (signum);

        if (atSignal != signals_.cend ()) {
            Registry::instance ().pop (signum, this);
            signals_.erase (atSignal);
        }

        return *this;
    }
}

