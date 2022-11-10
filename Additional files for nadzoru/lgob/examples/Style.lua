#! /usr/bin/env lua5.1

-- Font and color dialog selection demo. Also some runtime style changes.

require('lgob.loader')
require('lgob.gtk')
require('lgob.pango')

-- Create the widgets
local window = gtk.Window.new()
local vbox   = gtk.Box.new(gtk.ORIENTATION_VERTICAL  , 5)
local hbox   = gtk.Box.new(gtk.ORIENTATION_HORIZONTAL, 5)
local label  = gtk.Label.new("This is a sample text! Can the widgets style change?")
local bFont  = gtk.Button.new_with_mnemonic("Change _Font")
local bColor = gtk.Button.new_with_mnemonic("Change _Background")
local bReset = gtk.Button.new_with_mnemonic("_Reset")

-- Font dialog
local fontD  = gtk.FontChooserDialog.new("Font chooser", window)

-- Color dialog
local colorD = gtk.ColorChooserDialog.new("Color chooser", window)

-- Select font callback
function select_font()
    local res = fontD:run()
    fontD:hide()    
    
    if res == gtk.RESPONSE_OK then
        local desc = fontD:get_font()   
        label:override_font(pango.FontDescription.from_string(desc))
    end
end

-- Select background callback
function select_background()
    local res = colorD:run()
    colorD:hide()
    
    if res == gtk.RESPONSE_OK then
        local color = colorD:get('rgba')
        window:override_background_color(gtk.STATE_NORMAL, color)
    end
end

-- Reset callback
function reset()
    label:override_font()
    window:override_background_color(gtk.STATE_NORMAL)
    window:resize(1, 1)
end

-- Connect the callbacks
bFont:connect('clicked',  select_font)
bColor:connect('clicked', select_background)
bReset:connect('clicked', reset)
window:connect('delete-event', gtk.main_quit)

-- Add the widgets
hbox:pack_start(bFont , true, true, 0)
hbox:pack_start(bColor, true, true, 0)
hbox:pack_start(bReset, true, true, 0)
vbox:pack_start(label, true, true, 0)
vbox:pack_start(hbox, true, true, 0)
window:add(vbox)

-- Configure and show the window
window:set('title', "Style demo", 'window-position', gtk.WIN_POS_CENTER)
window:show_all()

gtk.main()
