/**
 * @file <client/js/main.js>
 * 
 * @author Prajankya Sonar - <prajankya@gmail.com>
 * 
 * @project ARGoS3-Webviz <https://github.com/NESTlab/argos3-webviz>
 * 
 * MIT License
 * Copyright (c) 2020 NEST Lab
 */


/* Define function to run after all files are loaded */
var onAllFilesLoaded = function () {

  /* Init KeyboardState */
  window.keyboard = new KeyboardState();
  
  /* Init client signals and flags */
  window.connectFlag = false;
  window.taskFlag = false;
  window.taskCommand = { command: 'task', signal: 'stop' };
  window.requestFlag = false;
  window.requestCommand = { command: 'request', number: 0 };
  window.sendFlag = false;
  window.sendCommand = { command: 'send', number: 0 };

  /* Generate uuid */
  window.client_id = generateRandomUUID();
  window.username = window.client_id.substring(0,8);
  console.log(window.client_id);

  window.target = ''; // Init robot selection to nothing
  window.connected = false; // Connected to a robot

  /* On Jquery load */
  $(function () {
    /* main panel-layout of the page */
    $('#layout').w2layout({
      name: 'app_layout',
      padding: 4,
      panels: [
        { type: 'top', size: 110, resizable: false },
        { type: 'left', size: "10%", resizable: true, content: 'left', hidden: true },
        { type: 'main', resizable: true, },
        { type: 'right', size: "30%", style: "background-color: #f2f2f2;border:0px", resizable: true, content: 'right' }
      ]
    });
    /* Log layout */
    $().w2layout({
      name: 'log_layout',
      padding: 5,
      panels: [{
        type: 'top',
        size: "50%",
        resizable: true,
        title: "Minimap",
        style: "padding:4px 8px;background:white",
        content: '<canvas></canvas>'
      },{
        type: 'main',
        size: "25%",
        resizable: true,
        title: "Log",
        style: "padding:4px 8px;background:white",
        content: '<div id="contentAreaLog" class="clusterize-content"></div>'
      }, {
        type: 'bottom',
        size: "25%",
        title: "LogErr",
        resizable: true,
        style: "padding:4px 8px;background:white",
        content: '<div id="contentAreaLogErr" class="clusterize-content"></div>'
      }]
    });
    /* Toolbar layout */
    $().w2layout({
      name: 'toolbar_layout',
      padding: 5,
      panels: [{
        type: 'top',
        size: "50%",
        resizable: false,
        // style: "padding:4px 8px;background:white",
        // content: '<div id="contentAreaLog" class="clusterize-content"></div>'
      }, {
        type: 'main',
        size: "50%",
        resizable: false,
        // style: "padding:4px 8px;background:white",
        // content: '<div id="contentAreaLogErr" class="clusterize-content"></div>'
      }]
    });

    /* Make them nested */
    w2ui['app_layout'].content('right', w2ui['log_layout']);
    w2ui['app_layout'].content('top', w2ui['toolbar_layout']);

    /* On Threejs panel Resize */
    w2ui['app_layout'].on('resize', function (event) {
      /* When resizing is complete */
      event.onComplete = function () {
        if (window.threejs_panel) {
          onThreejsPanelResize();
        }
      }
    });

    /* Load main logic code sub-files - sequentially */
    /* load threejs scene */
    loadJS("/js/three_scene.js", function () {
      /* Get the panel from layout */
      window.threejs_panel = $("#layout_app_layout_panel_main .w2ui-panel-content")

      /* Setup scene */
      IntializeThreejs(threejs_panel)
    }, true);

    /* Load websockets and connect to server */
    loadJS("/js/websockets.js", function () {

      /* Add styling for log divs */
      $("#layout_log_layout_panel_main>div.w2ui-panel-content")
        .attr("id", "scrollAreaLog")
        .addClass("clusterize-scroll")

      $("#layout_log_layout_panel_bottom>div.w2ui-panel-content")
        .attr("id", "scrollAreaLogErr")
        .addClass("clusterize-scroll")

      /* Initialize Log objects */
      window.log_clusterize = new Clusterize({
        show_no_data_row: false,
        scrollId: "scrollAreaLog",
        contentId: 'contentAreaLog'
      });

      window.logerr_clusterize = new Clusterize({
        show_no_data_row: false,
        scrollId: "scrollAreaLogErr",
        contentId: 'contentAreaLogErr'
      });

      let dropListTag = "".concat(
        "<select>"
        + "<option value='Select leader'>Select leader</option>"
        + "<option value='L1'>L1</option>"
        + "<option value='L2'>L2</option>"
        + "</select>"
      );

      let confirmButtonTag = "".concat(
        "<button type='button'>Confirm</button>"
      );

      let taskButtonTag = "".concat(
        "<button type='button'>START task</button>"
      );

      /* Add button on top panel */
      $("#layout_toolbar_layout_panel_top>div.w2ui-panel-content")
        .addClass('toolbar-flex-container')
        .append($("<div/>")
          .addClass('toolbar_counter')
          .attr("title", "Step counter")
          .prop("title", "Step counter")//for IE
          .html("{experiment.counter}")
        )
        /* Divider */
        .append($("<div/>")
          .addClass('toolbar_divider')
        )
        .append($("<div/>")
          .addClass('button')
          .addClass('icon-step')
          .attr("title", "Step experiment")
          .prop("title", "Step experiment")//for IE
          .click(function () {
            window.wsp.sendPacked({ command: 'step' })
          })
        )
        .append($("<div/>")
          .addClass('button')
          .addClass('icon-play')
          .attr('id', 'play_button')
          .attr("title", "Play experiment")
          .prop("title", "Play experiment")//for IE
          .click(function () {
            window.wsp.sendPacked({ command: 'play' })
          })
        )
        .append($("<div/>")
          .addClass('button')
          .addClass('icon-pause')
          .attr('id', 'pause_button')
          .attr("title", "Pause experiment")
          .prop("title", "Pause experiment")//for IE
          .click(function () {
            window.wsp.sendPacked({ command: 'pause' })
          })
        )
        .append($("<div/>")
          .addClass('button')
          .addClass('icon-ff')
          .attr('id', 'ff_button')
          .attr("title", "Fast forward experiment")
          .prop("title", "Fast forward experiment")//for IE
          .click(function () {
            var steps = parseInt($("#ff_steps_input").val());

            if (steps && steps >= 1 && steps <= 1000) {
              $("#ff_steps_input").val(steps)
              window.wsp.sendPacked({ command: 'fastforward', steps: steps })
            } else {
              window.wsp.sendPacked({ command: 'fastforward' })
            }
          })
        )
        .append($("<input/>")
          .attr('type', 'number')
          .attr('id', 'ff_steps_input')
          .attr('min', '1')
          .attr('max', '1000')
          .attr('value', '2')
          .attr("title", "Fast forward steps")
          .prop("title", "Fast forward steps")//for IE
        )
        /* Divider */
        .append($("<div/>")
          .addClass('toolbar_divider')
        )
        // .append($("<div/>")
        //   .addClass('button')
        //   .attr('id', 'stop_button')
        //   .addClass('icon-stop')
        //   .attr("title", "Terminate experiment")
        //   .prop("title", "Terminate experiment")//for IE
        //   .click(function () {
        //     // window.wsp.send('step')
        //   })
        // )
        // .append($("<div/>")
        //   .addClass('button')
        //   .addClass('icon-reset')
        //   .attr('id', 'reset_button')
        //   .attr("title", "Reset experiment")
        //   .prop("title", "Reset experiment")//for IE
        //   .click(function () {
        //     window.wsp.sendPacked({ command: 'reset' })
        //   })
        // )
        // /* Divider */
        // .append($("<div/>")
        //   .addClass('toolbar_divider')
        // )
        // .append($("<div/>")
        //   .addClass('button')
        //   .addClass('icon-settings')
        //   .attr('id', 'settings_button')
        //   .attr("title", "Settings")
        //   .prop("title", "Settings")//for IE
        //   .click(function () {
        //   })
        // )
        .append($("<div/>")
          .addClass('button')
          .addClass('icon-help')
          .attr("title", "Help")
          .prop("title", "Help")//for IE
          .click(function () {
            $("#HelpModal").w2popup({
              title: 'Help',
              showClose: true,
              height: 300,
              width: 500
            })
            // window.wsp.sendPacked({ command: 'reset' })
          })
        )

        /* Divider */
        .append($("<div/>")
          .addClass('toolbar_divider')
        )

        .append($("<div/>")
          .addClass("toolbar_status")
          .attr('id', 'name-label')
          .html("Username:")
        )

        .append($("<input/>")
          .attr('id', 'username_label')
          .attr('value', window.username)
          .attr('size', 10)
          .attr('maxlength', 16)
          .attr("title", "User name")
          .prop("title", "User name")//for IE
        )

        /* Divider */
        .append($("<div/>")
          .addClass('toolbar_divider')
        )

        .append($(dropListTag)
          .attr('id', 'leader_selected')
          .attr("title", "Select leader")
          .prop("title", "Select leader")//for IE
        )
        // .append($("<div/>")
        //   .addClass('button')
        //   .addClass('icon-select-leader')
        //   .attr('id', 'leader_select_button')
        //   .attr("title", "Connect to leader")
        //   .prop("title", "Connect to leader")//for IE
        //   .click(function () {
        //     let e = document.getElementById("leader_selected");
        //     let target = e.options[e.selectedIndex].text;

        //     window.wsp.sendPacked({ command: 'select_leader', robot: target });

        //     console.log("sent " + target);
        //   })
        // )
        /* Leader selection status */
        // .append($("<div/>")
        //   .addClass("toolbar_status")
        //   .html("{experiment.status}")
        // )

        .append($(confirmButtonTag)
          .attr('id','button_connect')
          .click(function () {
            let e_name = document.getElementById('username_label');
            window.username = e_name.value;
            let e_leader = document.getElementById('leader_selected');
            window.target = e_leader.options[e_leader.selectedIndex].text;

            if(window.target == 'Select leader') {
              window.target = '';
            }

            window.taskFlag = false;
            window.taskCommand = { command: 'task', signal: 'stop' };
            window.connectFlag = true;
            window.connectCommand = { command: 'select_leader' };
          })
        )

        .append($("<div/>")
          .addClass("toolbar_status")
          .attr('id', 'connection-status')
          .html("Disconnected")
        )

        /* Divider */
        .append($("<div/>")
          .addClass('toolbar_divider')
        )

        .append($(taskButtonTag)
          .attr('id','button_task')
          .click(function () {
            
            if(window.target != '') {
            
              if(window.taskCommand['signal'] == 'stop') {
                window.taskCommand['signal'] = 'start';
              } else if(window.taskCommand['signal'] == 'start') {
                window.taskCommand['signal'] = 'stop';  
              }

              window.taskFlag = true;
            }
          })
        )

        /* Spacer */
        .append($("<div/>").addClass('toolbar-spacer'))

        /* Right side of toolbar */
        .append($("<div/>")
          .addClass("toolbar_status")
          .html("{experiment.status}")
        )

      let requestButtonTag = "".concat(
        "<button type='button'>Request</button>"
      );

      let sendButtonTag = "".concat(
        "<button type='button'>Send</button>"
      );

      /* Add button on top panel */
      $("#layout_toolbar_layout_panel_main>div.w2ui-panel-content")
        .addClass('toolbar-flex-container')
        .append($("<input/>")
          .attr('type', 'number')
          .attr('id', 'request_input')
          .attr('min', '0')
          .attr('max', '1000')
          .attr('value', '0')
          .attr("title", "Number of robots to request")
          .prop("title", "Number of robots to request")//for IE
        )

        .append($(requestButtonTag)
          .attr('id','button_request')
          .attr("title", "Request robots")
          .prop("title", "Request robots")//for IE
          .click(function () {
            window.requestCommand['number'] = parseInt($("#request_input").val());
            window.requestFlag = true;
          })
        )

        /* Divider */
        .append($("<div/>")
          .addClass('toolbar_divider')
        )

        .append($("<input/>")
          .attr('type', 'number')
          .attr('id', 'send_input')
          .attr('min', '0')
          .attr('max', '1000')
          .attr('value', '0')
          .attr("title", "Number of robots to send")
          .prop("title", "Number of robots to send")//for IE
        )

        .append($(sendButtonTag)
          .attr('id','button_send')
          .attr("title", "Send robots")
          .prop("title", "Send robots")//for IE
          .click(function () {
            window.sendCommand['number'] = parseInt($("#send_input").val());
            window.sendFlag = true;
          })
        )

      window.experiment = {}

      /* Bind data using rivets */
      rivets.bind($('#experiment'), { experiment: window.experiment })

      $("#preloader").fadeOut()
      ConnectWebSockets()
    }, true);
  });
}

/* Load Jquery - sequentially */
loadJS("/js/libs/jquery.min.js", true)
loadJS("/js/libs/w2ui-1.5.rc1.min.js", true) /* Panels */
loadJS("/js/libs/clusterize.min.js", true) /* Better scroll for logs */
loadJS("/js/libs/jquery.contextMenu.min.js", true); /* Right click */

/* Load Websockets code */
loadJS("/js/libs/websocket-as-promised.js", true); /* basic websockets */
loadJS("/js/libs/robust-websocket.js", true); /* auto Reconnect */

/* Load Three.js code */
loadJS("/js/libs/three.min.js", true);
loadJS("/js/libs/OrbitControls.js", true);

loadJS("/js/libs/CSS2DRenderer.js", true);

loadJS("/js/libs/stats.min.js", true);
loadJS("/js/libs/GLTFLoader.js", true);

// From https://www.npmjs.com/package/@seregpie/three.text-sprite
loadJS("/js/libs/three.text-texture.js", true);
loadJS("/js/libs/three.text-sprite.js", true);

/* Load keyboard inputs code */
loadJS("/js/libs/KeyboardState.js", true);

/* Load uuid generator */
loadJS("/js/libs/generateRandomUUID.js", true);

/* Start running javascript after all files are loaded */
loadJS("/js/libs/rivets.bundled.min.js", onAllFilesLoaded, true);
