#! /usr/bin/env lua5.1
require('lgob.gtk')

local win = gtk.Window.new(gtk.WINDOW_TOPLEVEL)
win:set('title', 'Places', 'window-position', gtk.WIN_POS_CENTER)

local places = gtk.PlacesSidebar.new()
win:add(places)

win:connect('delete-event', gtk.main_quit)
win:show_all()
gtk.main()
