/*
 * LICENSE
 *
 * Copyright (c) 2017, David Daniel (dd), david@daniels.li
 *
 * handler.hpp is free software copyrighted by David Daniel.
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
#ifndef SIGNALS_HANDLER_INC
#define SIGNALS_HANDLER_INC

#include    <functional>
#include    <set>

namespace signals {

    /**
     * A signal handler that catches the signals from the operating system and safely
     * dispatches them to callbacks within the application context.
     *
     * The callback may take any desirable action except deleting the handler. It may
     * safely deregister signals in order to stop listening to them and to restore the
     * previous signal handler.
     *
     * If multiple signal handlers for the same signal are installed, the last set
     * handlers callback will be notified only. If a signal is deregistered from a
     * handler, the previously registered handler becomes active. The set of handlers for
     * a specific signal number form a stack wherein always the top of the stack gets
     * notifed of the signal only.
     */
    class Handler
    {
        public:
            /**
             * The signal handler callback
             *
             * Destroying the handler within the execution of the callback results in
             * undefined behaviour. It is however safe if the destruction of the handler
             * is done in a separate thread. One could start and detach a thread that
             * simply calls delete on the handler. A much better way is to use
             * ::signals::Handler::removeSignal that can safely be used within the
             * execution of the callback.
             */
            using Sink = std::function<void (int)>;

        public:
            /**
             * A handler needs a valid callback.
             */
            Handler () = delete;

            /**
             * @{
             * A handler cannot be copied but it can be moved.
             */
            Handler (const Handler&) = delete;
            Handler& operator= (const Handler&) = delete;

            Handler (Handler&&) = default;
            Handler& operator= (Handler&&) = default;
            /** @} */
            
            template <typename Callback>
                Handler (Callback&&);

            ~Handler ();

            Handler& addSignal (int);
            Handler& removeSignal (int);
            bool listens (int) const;

        private:
            void start (Sink&&);

        private:
            using Signals = std::set<int>;

        private:
            Signals signals_;
    };

    /**
     * Creates a signal handler.
     * @tparam a Callable with the signature of ::signals::Handler::Sink
     * @param callback the callback that takes the signal number as single argument 
     */
    template <typename Callback>
        inline Handler::Handler (Callback&& callback) :
            signals_ {}
        { start (Sink {std::forward<Callback> (callback)}); }

        /**
         * Returns true iff the given signal signum is listened to, iff the signal with
         * the given number is delivered to the callback.
         * @param signum the signal in question
         * @return true iff the given signal signum is listened to
         */
        inline bool Handler::listens (int signum) const
        { return signals_.find (signum) != signals_.cend (); }
}

#endif /* SIGNALS_HANDLER_INC */

