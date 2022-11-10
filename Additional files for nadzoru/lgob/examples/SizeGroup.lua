#! /usr/bin/env lua5.1

-- SizeGroup example

require('lgob.gtk')

local window = gtk.Window.new()
local vbox1 = gtk.Box.new(gtk.ORIENTATION_VERTICAL, 5)
local vbox2 = gtk.Box.new(gtk.ORIENTATION_VERTICAL, 5)
local vbox3 = gtk.Box.new(gtk.ORIENTATION_VERTICAL, 5)
local group1  = gtk.SizeGroup.new(gtk.SIZE_GROUP_HORIZONTAL)
local group2  = gtk.SizeGroup.new(gtk.SIZE_GROUP_HORIZONTAL)
local f1 = gtk.Frame.new("Option group 1")
local f2 = gtk.Frame.new("Option group 2")
local c, l, b = {}, {}, {}

local values = {
	"A string", "Another string", "Ahhh, this string!", "Boots!",
	"Hey Mary...", "Mouse", "A", "Great", "State", "of", "Random"
}

for i = 1, 5 do
	l[i] = gtk.Label.new("Option " .. i)
	c[i] = gtk.ComboBoxText.new()
	b[i] = gtk.Box.new(gtk.ORIENTATION_HORIZONTAL, 5)
	
	for j = 1, 5 do
		c[i]:append_text(values[((i + j) % #values) + 1])
	end
	
	c[i]:set('active', 0)
	b[i]:add(l[i], c[i])
	group1:add_widget(l[i])
	group2:add_widget(c[i])
end

vbox1:add(b[1], b[2], b[3])
vbox2:add(b[4], b[5])
f1:add(vbox1)
f2:add(vbox2)
vbox3:add(f1, f2)
window:add(vbox3)
window:set('title', 'SizeGroup', 'window-position', gtk.WIN_POS_CENTER)
window:connect('delete-event', gtk.main_quit)

window:show_all()
gtk.main()
