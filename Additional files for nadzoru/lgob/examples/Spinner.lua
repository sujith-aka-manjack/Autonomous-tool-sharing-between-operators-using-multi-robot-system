#! /usr/bin/env lua5.1
require('lgob.gtk')

local win       = gtk.Window.new()
local vbox      = gtk.Box.new(gtk.ORIENTATION_VERTICAL, 5)
local switch    = gtk.Switch.new()
local spinner   = gtk.Spinner.new()

vbox:pack_start(spinner, true , true, 0)
vbox:pack_start(switch,  false, false, 0 )
win:add(vbox)

win:set('title', "Spinner", 'window-position', gtk.WIN_POS_CENTER,
    'width-request', 100, 'height-request', 100)
win:connect('delete-event', gtk.main_quit)

switch:connect('notify::active', function ()
    local act = switch:get('active')
    spinner:set('active', act)
end)

win:show_all()
gtk.main()
