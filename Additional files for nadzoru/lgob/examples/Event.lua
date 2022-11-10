#! /usr/bin/env lua5.1	

-- Entry that only allow numbers!

require('lgob.gtk')
require('lgob.gdk')

local window = gtk.Window.new()
local vbox   = gtk.Box.new(gtk.ORIENTATION_VERTICAL, 5)
local label  = gtk.Label.new("Numbers only")
local entry  = gtk.Entry.new()

function keyPressed(data, event)
	local _, _, val = gdk.event_key_get(event)
	val = gdk.keyval_to_unicode(val)
	local num = val - string.byte('0')
	
	if (num >= 0 and num < 10) then
		return false
	else
		return true
	end
end

entry:connect('key-press-event', keyPressed)

vbox:add(label, entry)
window:add(vbox)
window:set('title', "Hello Event", 'window-position', gtk.WIN_POS_CENTER)
window:connect('delete-event', gtk.main_quit)

window:show_all()
gtk.main()
