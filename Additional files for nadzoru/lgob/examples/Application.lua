#! /usr/bin/env lua5.1

--[[
    This is not a simple example. Start with HelloWorld.lua :) .
--]]

require('lgob.gtk')
require('lgob.gobject')

local app    = gtk.Application.new('org.gtk.TestApplication', glib.APPLICATION_HANDLES_OPEN)
local window = nil

function create()
    if window then
        window:present()
        print('App is already running')
    else
        window = gtk.Window.new(gtk.WINDOW_TOPLEVEL)
        window:connect('delete-event', gtk.main_quit)
        window:set('application', app, 'window-position', gtk.WIN_POS_CENTER)
        window:show_all()
        print('Creating the main instance')
    end
end

-- when running without parameters, i. e., ./Application
app:connect('activate', function(...)
    print('activate signal')
    create()
end)

-- when running with parameters, i. e., ./Application file1.lua "second file"
app:connect('open', function(ud, files, nfiles)
    print('open signal')
    print(files, nfiles)
    create()
end)

table.insert(arg, 0, 'lgob')
app:run(arg)
