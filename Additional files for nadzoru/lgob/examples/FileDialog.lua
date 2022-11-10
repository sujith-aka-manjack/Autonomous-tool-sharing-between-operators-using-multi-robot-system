#! /usr/bin/env lua5.1

-- FileDialog demo

require('lgob.gtk')

function run_dialog(dialog)
	dialog:run()
	dialog:hide()
	
	local names = dialog:get_filenames()
	for i = 1, #names do
		print(names[i])
	end
end

local window = gtk.Window.new()
local button = gtk.Button.new_with_mnemonic("_Show Dialog")
local dialog = gtk.FileChooserDialog.new("Select the file", window, gtk.FILE_CHOOSER_ACTION_OPEN,
	'gtk-cancel', gtk.RESPONSE_CANCEL, 'gtk-ok', gtk.RESPONSE_OK)

filter = gtk.FileFilter.new()
filter:add_pattern('*.lua')
filter:set_name("Lua files")

dialog:add_filter(filter)
dialog:set('select-multiple', true)
window:connect('delete-event', gtk.main_quit)
button:connect('clicked', run_dialog, dialog)

window:set('title', "FileDialog demo", 'window-position', gtk.WIN_POS_CENTER)
window:add(button)
window:show_all()

gtk.main()
