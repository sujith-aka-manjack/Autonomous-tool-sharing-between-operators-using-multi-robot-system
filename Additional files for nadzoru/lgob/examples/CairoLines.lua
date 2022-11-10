#! /usr/bin/env lua5.1

-- Lines! Suports window resize.

require('lgob.gtk')
require('lgob.gdk')
require('lgob.cairo')

function draw(widget, cr)
    local width, height = widget:get_allocated_size()
    cr = cairo.Context.wrap(cr)

    -- Configure
    cr:set_source_rgb(0.1, 0.1, 1)
    cr:set_line_cap(cairo.LINE_CAP_ROUND)
    cr:set_line_width(10)

    -- Draw some lines
    local h3 = height / 3

    cr:move_to(20, 20)
    cr:line_to(width, h3)
    cr:rel_line_to(-width, h3)
    cr:rel_line_to(width, h3)
    cr:stroke()

    cr:set_source_rgb(1, 0.1, 0.1)
    cr:set_line_cap(cairo.LINE_CAP_BUTT)
    cr:set_line_width(5)

    cr:move_to(width - 20, height - 20)
    cr:line_to(0, 0)
    cr:line_to(0, height)
    cr:line_to(width, 0)
    cr:stroke()
    
    return false
end

local window = gtk.Window.new()
local area   = gtk.DrawingArea.new()
window:add(area)
window:set('title', "Lines demo", 'window-position', gtk.WIN_POS_CENTER)
window:show_all()

window:connect('delete-event', gtk.main_quit)
area:connect('draw', draw, area)

gtk.main()
