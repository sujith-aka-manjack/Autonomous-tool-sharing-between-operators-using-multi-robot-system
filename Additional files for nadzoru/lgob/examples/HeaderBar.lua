#! /usr/bin/env lua5.1

require('lgob.gtk')

local win = gtk.Window.new(gtk.WINDOW_TOPLEVEL)
win:set('window-position', gtk.WIN_POS_CENTER, 
'title', 'HeaderBar demo', 'width-request', 500, 'height-request', 300)
    
local header  = gtk.HeaderBar.new()
local content = gtk.TextView.new()
content:get_buffer():set_text("My application content.", -1)

header:set_title('My title')
header:set_subtitle('This great subtitle is great.')
header:set_show_close_button(true)

local scroll = gtk.ScrolledWindow.new()
scroll:add(content)
win:set_titlebar(header)
win:add(scroll)

win:connect('delete-event', gtk.main_quit)
win:show_all()
gtk.main()
