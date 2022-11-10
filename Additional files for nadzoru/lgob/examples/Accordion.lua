#! /usr/bin/env lua5.1

-- Lua port of the 'CSS Accordion' program from gtk3-demo.

require('lgob.gtk')

-- apply a css style to a widget
function apply_css(widget, provider)
    local ctx      = gtk.Widget.get_style_context(widget)
    local priority = 1000
    ctx:add_provider(provider, priority)
end

local window = gtk.Window.new(gtk.WINDOW_TOPLEVEL)
window:set('default-width', 600, 'default-height', 300)
window:connect('destroy', gtk.main_quit)

local box = gtk.Box.new(gtk.ORIENTATION_HORIZONTAL, 0)
box:set('halign', gtk.ALIGN_CENTER, 'valign', gtk.ALIGN_CENTER)
window:add(box)

local children = {}
table.insert(children, gtk.Button.new_with_label("This"))
table.insert(children, gtk.Button.new_with_label("Is"))
table.insert(children, gtk.Button.new_with_label("A"))
table.insert(children, gtk.Button.new_with_label("CSS"))
table.insert(children, gtk.Button.new_with_label("Accordion"))
table.insert(children, gtk.Button.new_with_label(":-)"))

local css      = require('AccordionCSS')
local provider = gtk.CssProvider.new()
provider:load_from_data(css, #css)

for i = 1, #children do
    local child = children[i]
    box:add(child)
    apply_css(child, provider)    
end

apply_css(window, provider)
window:show_all()

gtk.main()
