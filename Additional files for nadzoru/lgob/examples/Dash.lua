#! /usr/bin/env lua5.1

-- Hello World

require('lgob.gtk')
require('lgob.cairo')

function draw(area, cr)
	cr = cairo.Context.wrap(cr)
	local dashes = {1, 2, 4, 8, 16}
	local w, h = area:get_allocated_size()
	
	cr:set_source_rgb(0, 0, 1)
	cr:set_dash(dashes, 5, 10)
	cr:set_line_width(1.5)
	
	cr:move_to(0, 0)
	cr:line_to(w, h)
	cr:stroke()
	
	cr:set_dash(dashes, 3, 10)
	cr:move_to(w, 0)
	cr:line_to(0, h)
	cr:set_source_rgb(1, 0, 0)
	cr:stroke()
end

local window = gtk.Window.new()
local area   = gtk.DrawingArea.new()
window:add(area)
window:set('title', "Hello World", 'window-position', gtk.WIN_POS_CENTER)

window:connect('delete-event', gtk.main_quit)
area:connect('draw', draw, area)

window:show_all()
gtk.main()
