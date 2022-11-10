#! /usr/bin/env lua5.1

require('lgob.gtk')

local win  = gtk.Window.new()
win:set('title', 'Grid demo', 'window-position', gtk.WIN_POS_CENTER)
win:connect('delete-event', gtk.main_quit) 

local grid = gtk.Grid.new()
local l1   = gtk.Label.new('Hey')
local l2   = gtk.Label.new('You')
local l3   = gtk.Label.new('Resize us!')

l1:set('hexpand', true, 'vexpand', true)
l2:set('hexpand', true, 'vexpand', true)
l3:set('hexpand', true, 'vexpand', true)

local b1   = gtk.Button.new_with_label('Button 1')
local b2   = gtk.Button.new_with_label('Button 2')

b1:set('valign', gtk.ALIGN_CENTER)
b2:set('valign', gtk.ALIGN_FILL)

win:add(grid)
grid:attach(l1, 0, 0, 1, 1)
grid:attach(l2, 1, 0, 1, 1)
grid:attach(l3, 0, 1, 2, 1)
grid:set('row-spacing', 5)

grid:attach_next_to(b1, l2, gtk.POS_RIGHT , 1, 1)
grid:attach_next_to(b2, l3, gtk.POS_RIGHT , 1, 1)

win:show_all()

gtk.main()
