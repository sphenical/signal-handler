# signal-handler

![Build Status](https://sphenic.ch/jenkins/buildStatus/icon?job=signal-handler)

Applications often need a way to properly shutdown before execution ends. This includes
closing of file descriptors, joining running threads and running all other kind of cleanup
tasks. If the order in which the various subtasks are executed is relevant, this can be a
difficult task since simple in-order destruction of the involved objects can be different.
One solution to this problem is to register a signal handler that catches the signal from
the operating system and makes sure, the application is notified and can gracefully
shutdown. Within a signal handler only a limited number of things can be done, the handler
is executed in a special context. It is therefore required to postpone a corresponding
signalling to the application until the main process joins the control flow again so it is
possible to run any action that needs to be running within the application context, just
like the Top-part of the Top-Halves.

This library wraps handling of the POSIX signals and provides an object oriented method
that allows the user to register signal handlers that receive the corresponding signal
within the application context. The library installs a separate thread that is used to
report the signals.

## Usage

A signal handler is constructed with a callback that is executed on every registered
signal number and has the following signature and gets the signal number passed as single
argument:
```cpp
void handler (int);
```

Internally the callback is stored in a function object with the following type:
```cpp
std::function<void (int)>
```
Anything convertible to this type can be provided as a constructor argument to a signal
handler.

The total of all handlers is stored on a stack per signal number, only the top handler is
executed if the corresponding signal is emitted. If the top handler for a given signal is
removed, the next top handler on the stack gets active until there is no more handler.

A typical usage scenario looks like the following example.
```cpp
#include <sig/handler.hpp>
#include <csignal>

#include "MyApp.hpp"

int main ()
{
    MyApp myApp;
    ...
    sig::Handler handler {
        std::bind (
            &MyApp::onSignal, &myApp, std::placeholders::_1)};

    handler
        .addSignal (SIGINT)
        .addSignal (SIGTERM);

    return myApp.run ();
}
```

The executable has to be linked with the signal handler library.

In order to use the library embedded within your application - this is recommended since
the library consists of one single source file - you can add the following to your
`CMakeLists.txt`, assuming that this repository has been checked out under
`vendor/signal-handler`:

```cmake
set (SIGNALHANDLER_BUILD_SHARED_LIBS OFF CACHE BOOL "Don't build the signal handler shared library.")
set (SIGNALHANDLER_DISABLE_INSTALLS ON CACHE BOOL "Don't install the signal handler library.")
add_subdirectory (vendor/signal-handler)

...
# For instance an application myapp:
add_executable (myapp)
target_sources (myapp
    PRIVATE
        MyApp.cpp
)
..
target_link_libraries (myapp signalhandler::signalhandler)
```

In order to checkout the library on the subfolder `vendor/` as sub tree for instance, the
following can be issued:
```git
git remote add signal-handler https://github.com/sphenical/signal-handler.git
git subtree add --prefix vendor/signal-handler signal-handler master --squash
```

