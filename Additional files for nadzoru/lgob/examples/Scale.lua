#! /usr/bin/env lua5.1

-- Scale widgets

require('lgob.gtk')

local window = gtk.Window.new()
local scale1 = gtk.Scale.new_with_range(gtk.ORIENTATION_VERTICAL, 1, 100, 10)
local adjust = gtk.Adjustment.new(1, 1, 100, 2, 10, 0)
local scale2 = gtk.Scale.new(gtk.ORIENTATION_HORIZONTAL, adjust)
local vbox   = gtk.Box.new(gtk.ORIENTATION_VERTICAL, 5)

vbox:pack_start(scale1, true, true, 0)
vbox:pack_start(scale2, true, true, 0)
window:add(vbox)

window:set('title', "Hello World", 'window-position', gtk.WIN_POS_CENTER)
window:set('width-request', 200, 'height-request', 200)

window:connect('delete-event', gtk.main_quit)
window:show_all()
gtk.main()
