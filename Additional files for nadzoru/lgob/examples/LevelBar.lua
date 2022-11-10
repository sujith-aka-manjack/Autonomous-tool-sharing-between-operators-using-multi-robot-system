#! /usr/bin/env lua5.1

require('lgob.gtk')

local win = gtk.Window.new(gtk.WINDOW_TOPLEVEL)
win:connect('delete-event', gtk.main_quit)
win:set(
    'window-position', gtk.WIN_POS_CENTER,
    'width-request'  , 300,
    'height-request' , 30
)

local level = gtk.LevelBar.new()
level:set_value(0.6)
win:add(level)
win:show_all()

gtk.main()
