#! /usr/bin/env lua5.1

require('lgob.gtk')

local win = gtk.Window.new(gtk.WINDOW_TOPLEVEL)
win:set('window-position', gtk.WIN_POS_CENTER, 'title', 'ActionBar demo',
    'width-request', 500, 'height-request', 300)
    
local box     = gtk.Box.new(gtk.ORIENTATION_VERTICAL)
local action  = gtk.ActionBar.new()
local content = gtk.TextView.new()
content:get_buffer():set_text("My application content.", -1)

action:pack_start( gtk.Label.new("Start") )
action:pack_end  ( gtk.Label.new("End 2") )
action:pack_end  ( gtk.Label.new("End 1") )
action:set_center_widget( gtk.Button.new_with_mnemonic("_Central") )

box:pack_start(action, false, true, 5)
box:pack_start(content, true, true, 5)
win:add(box)

win:connect('delete-event', gtk.main_quit)
win:show_all()
gtk.main()
