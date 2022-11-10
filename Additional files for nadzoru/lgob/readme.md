About lgob
==========

Description
-----------
    
lgob provides bindings of GObject-based libraries (like like GTK+ and
WebKitGTK+), for Lua 5.1 / 5.2 / LuaJIT. It consists of a compiler that parses
[GObject-Instrospection][1] _gir_ files and generates Lua modules.
lgob ships with bindings for GTK+, pango, cairo, vte, WebKitGtk, GtkTextView,
and others.

lgob can be compiled and used in MS-Windows. There are official builds in [2].

Sample usage (included modules)
===============================

Hello GTK+
----------

    require('lgob.gtk')
    
    local window = gtk.Window.new(gtk.WINDOW_TOPLEVEL)
    window:connect('delete-event', gtk.main_quit)
    
    local button = gtk.Button.new_with_label("Click me")
    button:connect('clicked', function() print("clicked") end )
    
    window:add(button)
    window:show_all()
    gtk.main()

    
Usage (as a compiler)
=====================

Basic
-----

    lgob-gir-parser.lua -i myinput.gir -o mydef.def -l mylog1
    lgob-generator.lua  -i mydef.def   -o myiface.c -l mylog2 -v 3.2

Definition files
----------------

The files used as input to the compiler. You can use convert .gir
files to .def files using lgob-gir-parser.

It's possible to merge definition files by doing a simple
dofile('name.def').

Versioning
----------

The .def files can carry 'since' and 'deprecated' version
information, to allow the code generation target a specific
version (passing -v 3.12, would generate code for functions available
from 3.0 to 3.12, excluding the functions deprecated
between 3.0 and 3.12.

The gobject-instrospection .gir format does not gives version 
information on classes and enums, and not all functions have version
annotations. For that, it's possible to override some information
with the .ovr files.

License
=======

LGPL v3.
    
Install
=======

You can build and install lgob by calling

    $ make
    # make install PREFIX=/usr/local

You may need to adjust config.lua to change compiler, environment
options, and select the target Lua version.

You can also select what modules to compile (or compile individual modules)
by using/editing the build.lua and build_all.lua scripts located in the
root directory.

The first version of lgob was released around 2008. lgob works with Lua 5.1, Lua 5.2,
and LuaJIT.

Dependencies
------------

On Debian/Ubuntu you will need the following packages to compile all
lgob modules:

liblua5.1-0-dev, libgtk-3-dev, libgtk-3-dev, libgstreamer0.10-dev, libgstreamer-plugins-base0.10-dev,
libwebkitgtk-3.0-dev, libvte-2.90-dev, libpoppler-glib-dev, libpango1.0-dev, libgtksourceview-3.0-dev,
libglib2.0-dev, libcairo2-dev, libpango1.0-dev, libpoppler-dev, libatk1.0-dev

[1]: http://wiki.gnome.org/GObjectIntrospection
[2]: https://bitbucket.org/lucashnegri/lgob/downloads
