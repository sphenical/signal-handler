/*
 * LICENSE
 *
 * Copyright (c) 2017, David Daniel (dd), david@daniels.li
 *
 * main.cpp is free software copyrighted by David Daniel.
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
#include    <cstdlib>
#include    <iostream>

#include    <signals/handler.hpp>
#include    <csignal>

#include    <atomic>
#include    <thread>
#include    <chrono>
#include    <memory>

static std::atomic_bool running {true};
static std::unique_ptr<signals::Handler> secondHandler;

static void staticHandlerFunc (int);
static signals::Handler staticHandler {&staticHandlerFunc};

void staticHandlerFunc (int signum)
{
    std::cout
        << "In static handler, received signal: "
        << signum << std::endl;

    staticHandler.removeSignal (signum);
}

void receiveSecondHandlerSig (int signum)
{
    std::cout
        << "In second handler, received signal: "
        << signum << std::endl;

    secondHandler.reset (nullptr);
}

void receiveOtherHandlerSig (int signum, signals::Handler& handler)
{
    std::cout
        << "In other handler, received signal: "
        << signum << std::endl;

    handler.removeSignal (signum);
}

void receiveSignalInApp (int signum)
{
    std::cout << "received signal: " << signum << std::endl;
    running.exchange (false);
}

void wasteTime ()
{
    while (running.load ()) {
        std::this_thread::sleep_for (std::chrono::seconds {2});
    }

    std::cout << "not running any more...\n";
}

int main ()
{
    std::thread worker {&wasteTime};
    signals::Handler handler {&receiveSignalInApp};

    handler
        .addSignal (SIGTERM)
        .addSignal (SIGINT);

    secondHandler.reset (
            new signals::Handler {&receiveSecondHandlerSig});

    secondHandler
        ->addSignal (SIGTERM);
        // .addSignal (SIGINT);

    signals::Handler otherHandler {
        std::bind (
                &receiveOtherHandlerSig,
                std::placeholders::_1,
                std::ref (otherHandler))};
    otherHandler
        .addSignal (SIGINT);
        // .addSignal (SIGTERM);

    secondHandler->addSignal (SIGINT);
    otherHandler.addSignal (SIGTERM);

    staticHandler
        .addSignal (SIGTERM)
        .addSignal (SIGINT);

    worker.join ();

    return EXIT_SUCCESS;
}

