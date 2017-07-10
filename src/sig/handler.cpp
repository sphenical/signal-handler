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
#include    <sig/handler.hpp>

#include    <cstdint>
#include    <limits>
#include    <cstring>
#include    <vector>
#include    <map>
#include    <stdexcept>
#include    <chrono>
#include    <thread>
#include    <atomic>
#include    <set>

#include    <unistd.h>
#include    <sys/select.h>
#include    <signal.h>
#include    <fcntl.h>

namespace sig {

    namespace {
        using Signals = std::set<int>;
        using Sink = Handler::Sink;
    }

    struct Handler::Handle
    {
        Signals sigs;
    };

    namespace {
        using Handle = Handler::Handle;

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
            const Handle* handle;

            Dispatcher (const Handle*);
        };

        Dispatcher::Dispatcher (const Handle* h) :
            ancestor {},
            handle {h}
        {}

        class Registry
        {
            using Dispatchers = std::vector<Dispatcher>;
            using Handlers = std::map<int, Dispatchers>;
            using Callables = std::map<const Handle*, Sink>;

            public:
                static Registry& instance ();
                ~Registry ();

                template <typename Func>
                    Registry& addHandler (const Handle*, Func&&);

                Registry& removeHandler (const Handle*);

                Registry& push (int, const Handle*);
                Registry& pop (int, const Handle*);

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
            inline Registry& Registry::addHandler (const Handle* handle, Func&& func)
            {
                callables_.emplace (handle, std::forward<Func> (func));
                return *this;
            }

        inline Registry& Registry::removeHandler (const Handle* handle)
        {
            callables_.erase (handle);
            return *this;
        }

        Registry& Registry::push (int signum, const Handle* handle)
        {
            struct sigaction action;

            std::memset (&action, 0, sizeof action);

            action.sa_handler = &catch_signal;
            action.sa_flags = 0;
            sigfillset (&action.sa_mask);

            Dispatcher dispatcher {handle};

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

        Registry& Registry::pop (int signum, const Handle* handle)
        {
            Handlers::iterator atHandler = handlers_.find (signum);
            if (atHandler != handlers_.end ()) {
                Dispatchers& dispatchers = atHandler->second;
                Dispatchers::reverse_iterator atDispatcher = dispatchers.rbegin ();
                Dispatchers::reverse_iterator endOfDispatchers = dispatchers.rend ();

                bool onTop = true;
                while (onTop && atDispatcher != endOfDispatchers) {
                    if (atDispatcher->handle == handle) {

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
                    if (atDispatcher->handle == handle) {

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
                        else if (signum != ShutdownSignal && running_.load ()) {
                            readErrors = 0;
                            Handlers::iterator atHandler = handlers_.find (signum);

                            if (atHandler != handlers_.end ()) {
                                Dispatchers& dispatchers = atHandler->second;

                                if (!dispatchers.empty ()) {
                                    const Handle* handle = dispatchers.back ().handle;

                                    Callables::const_iterator atCallable = callables_.find (handle);
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
     * Creates a signal handler.
     * Registers the handler in the internal registry.
     * @param callback the callback that takes the signal number as single argument 
     */
    Handler::Handler (Sink&& callback) :
        handle_ {new Handle}
    {
        Registry::instance ()
            .addHandler (handle_.get (), std::move (callback));
    }

    Handler::Handler (Handler&&) = default;
    Handler& Handler::operator= (Handler&&) = default;

    /**
     * Unregisters the handler from the internal registry.
     */
    Handler::~Handler ()
    {
        Registry& registry = Registry::instance ();

        for (const auto s : handle_->sigs) {
            registry.pop (s, handle_.get ());
        }

        registry.removeHandler (handle_.get ());
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
        Signals::const_iterator atSignal = handle_->sigs.find (signum);

        if (atSignal == handle_->sigs.cend ()) {
            Registry::instance ().push (signum, handle_.get ());
            handle_->sigs.insert (signum);
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
        Signals::const_iterator atSignal = handle_->sigs.find (signum);

        if (atSignal != handle_->sigs.cend ()) {
            Registry::instance ().pop (signum, handle_.get ());
            handle_->sigs.erase (atSignal);
        }

        return *this;
    }

    /**
     * Returns true iff the given signal signum is listened to, iff the signal with
     * the given number is delivered to the callback.
     * @param signum the signal in question
     * @return true iff the given signal signum is listened to
     */
    bool Handler::listensOn (int signum) const
    { return handle_->sigs.find (signum) != handle_->sigs.cend (); }
}

