#! /usr/bin/env lua5.1

-- About dialog

require("lgob.gtk")
require("lgob.gdk")

-- Create the widgets
local window = gtk.Window.new()
local dialog = gtk.AboutDialog.new()
local button = gtk.Button.new_with_mnemonic("_About")
local logo   = gdk.Pixbuf.new_from_file("icon.png")

function showAbout()
    dialog:run()
    dialog:hide()
end

dialog:set("program-name", "AboutDialog demo", "authors", {
    "Lucas Hermann Negri <lucashnegri@gmail.com>",
    "Hans Elbers",
    "Audren Cezar",
    "jf Cap",
    "Yuri K. Lopes",
    "daurnimator",
    "Martin",
    "Steve Starr",
    "Iñigo Serna",
    "Natanael Copa",
    "Fredy Paquet",
    "Kun Wang",
    "speps",
    "Bertrand Mansion"
},
"comments", "No comments!", "license", "LGPL 3+", "logo", logo, "title", "About...",
"website", "https://bitbucket.org/lucashnegri/lgob")

window:add(button)
window:set("title", "About Dialog", "window-position", gtk.WIN_POS_CENTER)

window:connect("delete-event", gtk.main_quit)
button:connect("clicked", showAbout)

window:show_all()
gtk.main()
