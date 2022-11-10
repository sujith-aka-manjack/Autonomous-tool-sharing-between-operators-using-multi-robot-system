#! /usr/bin/env lua5.1

-- Ported from Pango manual

require('lgob.gtk')
require('lgob.cairo')
require('lgob.pango')
require('lgob.pangocairo')

local N_WORDS = 10 - 1
local desc = pango.FontDescription.from_string('Sans Bold 27')

function draw(area, cr)
	cr = cairo.Context.wrap(cr)
	local width, height = area:get_allocated_size()
	local RADIUS = (width < height and width or height) / 2

	local layout = pangocairo.create_layout(cr)
	layout:set_text("Text", -1)
	layout:set_font_description(desc)
	cr:translate(RADIUS, RADIUS)

	-- Draw the layout N_WORDS times in a circle
  	for i = 0, N_WORDS do
		local angle = (360 * i) / N_WORDS
		cr:save()

		-- Color gradient
      	local red = (1 + math.cos ((angle - 60) * math.pi / 180)) / 2;
		cr:set_source_rgb(red, 0, 1 - red)
      	cr:rotate(angle * math.pi / 180);
    
		-- Inform Pango to re-layout the text with the new transformation
      	pangocairo.update_layout(cr, layout)
		local width, height = layout:get_size()
		cr:move_to(- (width / pango.SCALE) / 2, - RADIUS)
      	pangocairo.show_layout(cr, layout)

      	cr:restore()
	end
end

local window = gtk.Window.new()
local area   = gtk.DrawingArea.new()
window:add(area)
window:set('title', "Pango demo", 'window-position', gtk.WIN_POS_CENTER,
'width-request', 300, 'height-request', 300)

window:connect('delete-event', gtk.main_quit)
area:connect('draw', draw, area)

window:show_all()
gtk.main()
