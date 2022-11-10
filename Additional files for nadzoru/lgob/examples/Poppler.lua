#! /usr/bin/env lua5.1

require('lgob.gtk')
require('lgob.cairo')
require('lgob.poppler')

if not arg[1] then
    error("Usage: " .. arg[0] .. " uri")
end

local doc, err, msg = poppler.Document.new_from_file(arg[1])
assert(doc, msg)

local page = doc:get_page(0)
assert(page, "Couldn't open the first page")

local pages = doc:get_n_pages()
print("There are " .. pages .. " pages in this file")

local win = gtk.Window.new(gtk.WINDOW_TOPLEVEL)
win:connect('delete-event', gtk.main_quit)
win:connect('draw', page.render, page)

win:set('title', "Poppler example", 'window-position', gtk.WIN_POS_CENTER,
    'app-paintable', true, 'width-request', 600, 'height-request', 800)
    
win:show_all()
gtk.main()
