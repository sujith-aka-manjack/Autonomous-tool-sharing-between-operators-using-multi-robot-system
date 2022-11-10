#! /usr/bin/env lua5.1
require('lgob.cairo')

local width, height = 300, 300
local surface = cairo.ImageSurface.create(cairo.FORMAT_ARGB32, width, height)
local cr = cairo.Context.create(surface)

local text = "cairo"
cr:select_font_face("Sans", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_NORMAL)
cr:set_font_size(100.0)

extents = cairo.TextExtents.create()
cr:text_extents(text, extents)

local x_bearing, y_bearing, width, height, x_advance, y_advance = extents:get()

local x = 25.0
local y = 150.0

cr:move_to(x, y)
cr:show_text(text)

cr:set_source_rgba (1, 0.2, 0.2, 0.6)
cr:set_line_width(6.0)
cr:arc(x, y, 10.0, 0, 2 * math.pi)
cr:fill()
cr:move_to(x, y)
cr:rel_line_to(0, -height)
cr:rel_line_to(width, 0)
cr:rel_line_to(x_bearing, -y_bearing)
cr:stroke()

surface:write_to_png("cairotextextents.png")

cr:destroy()
surface:destroy()
