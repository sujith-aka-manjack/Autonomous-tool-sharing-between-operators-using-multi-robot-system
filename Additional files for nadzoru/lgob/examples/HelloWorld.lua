#! /usr/bin/env lua5.1

require('lgob.gtk')

local window = gtk.Window.new(gtk.WINDOW_TOPLEVEL)
window:connect('delete-event', gtk.main_quit)
window:show_all()
gtk.main()
