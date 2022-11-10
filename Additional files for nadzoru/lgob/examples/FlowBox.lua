#! /usr/bin/env lua5.1

require('lgob.gtk')

local win = gtk.Window.new(gtk.WINDOW_TOPLEVEL)
win:set('window-position', gtk.WIN_POS_CENTER, 
    'title', 'FlowBox demo', 'default-width', 500, 'default-height', 300)
    
local box = gtk.FlowBox.new()
box:set("homogeneous", true)
box:set("max-children-per-line", 20)

for i = 1, 100 do
    box:add( gtk.Button.new_with_label(tostring(i)))
end

local scroll = gtk.ScrolledWindow.new()
scroll:add(box)
win:add(scroll)
win:connect('delete-event', gtk.main_quit)
win:show_all()
gtk.main()
