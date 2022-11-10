#! /usr/bin/env lua5.1

require('lgob.gtk')

local window   = gtk.Window.new(gtk.WINDOW_TOPLEVEL)
window:set('width-request', 200, 'height-request', 75, 'window-position',
    gtk.WIN_POS_CENTER, 'title', 'Revealer')
    
local box = gtk.Box.new(gtk.ORIENTATION_HORIZONTAL)
local btn = gtk.Button.new_with_label("Do!")

local label    = gtk.Label.new("Surprised?")
local revealer = gtk.Revealer.new()

revealer:add(label)
revealer:set_transition_duration(2000)
revealer:set_transition_type(gtk.REVEALER_TRANSITION_TYPE_CROSSFADE)

btn:connect('clicked', function() revealer:set_reveal_child(true) end)
box:pack_start(revealer, true, true , 10)
box:pack_start(btn     , true, false, 10)

window:connect('delete-event', gtk.main_quit)
window:add(box)
window:show_all()
gtk.main()
