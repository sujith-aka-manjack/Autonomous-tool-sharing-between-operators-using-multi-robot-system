#! /usr/bin/env lua5.1

-- Cairo toy text example

require('lgob.gtk')
require('lgob.cairo')

function draw(area, cr)
    cr = cairo.Context.wrap(cr)
    local width, height = area:get_allocated_size()
    
    cr:select_font_face('sans', cairo.FONT_SLANT_OBLIQUE, cairo.FONT_WEIGHT_BOLD)
    cr:set_font_size(14)
    cr:move_to(20, 20)
    cr:show_text("Hello! This is an example of the cairo toy text API")
    cr:move_to(20, 40)
    cr:show_text("For a serious work, use pango to handle text!")
    
    return false
end

local window = gtk.Window.new()
local area   = gtk.DrawingArea.new()
window:add(area)
window:set('title', "Cairo Toy Text", 'window-position', gtk.WIN_POS_CENTER,
'width-request', 450, 'height-request', 50)

window:connect('delete-event', gtk.main_quit)
area:connect('draw', draw, area)

window:show_all()
gtk.main()
