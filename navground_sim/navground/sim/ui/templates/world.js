var svg;
var svg_world;

{% if notebook %}

// require.config({
//   paths: {
//     'svg-pan-zoom': 'https://cdnjs.cloudflare.com/ajax/libs/svg.pan-zoom.js/2.7.0/svg.pan-zoom.min.js'
//   }
// })

// require(["svg-pan-zoom"], function(svgPanZoom) {
//   window.svgPanZoom = svgPanZoom('#{{prefix}}svg', {
//     panEnabled: true,
//     controlIconsEnabled: true,
//     zoomEnabled: true,
//     fit: true,
//     maxZoom: 200,
//     minZoom: 0.1
//   });
//   svg = document.getElementsByClassName("svg-pan-zoom_viewport")[0];
//   svg_world = document.getElementById("{{prefix}}world");
// });

{% endif %}

{% if not notebook %}
// window.onload = function() {
//   window.svgPanZoom = svgPanZoom('#svg', {
//     panEnabled: true,
//     controlIconsEnabled: true,
//     zoomEnabled: true,
//     fit: true,
//     maxZoom: 40,
//     minZoom: 0.1
//   });
//   svg = document.getElementsByClassName("svg-pan-zoom_viewport")[0];
//   svg_world = document.getElementById("world");
//   window.requestAnimationFrame(update);
// }
{% endif %}

{% if with_websocket %}

var ws_address = "ws://127.0.0.1:{{port}}/"
var ws = new WebSocket(ws_address);

ws.onopen = function(event) {
  console.log('{{prefix}} Opened Web socket ' + ws_address)
};

ws.onmessage = function(event) {
  var [type, data] = JSON.parse(event.data)
  switch (type) {
    case 'm':
      move(data)
      break;
    case 'r':
      reset(data)
      break;
    case 's':
      set(data)
      break;
    case 'v':
      view(data)
      break;
    case '+':
      add(data)
      break;
    default:
      break
  }
}
{% endif %}

var agents = new Set();
var entity = {};
var NS = "http://www.w3.org/2000/svg";
var prototypes = {'': '#{{prefix}}agent', 'thymio': '#{{prefix}}thymio'}

function add_wall(id, points) {
  var e = document.createElementNS(NS,"polyline");
  var ps = []
  for (var i in points) {
     ps.push(`${points[i][0]},${points[i][1]}`)
  } 
  e.setAttribute("points", ps.join())
  e.setAttribute("class", "wall");
  e.setAttribute("id", id);
  return e
}

function add_obstacle(id, point, radius) {
  var e = document.createElementNS(NS,"circle");
  e.setAttribute("cx", `${point[0]}`)
  e.setAttribute("cy", `${point[1]}`)
  e.setAttribute("r", `${radius}`)
  e.setAttribute("class", "obstacle");
  e.setAttribute("id", id);
  return e
}

function add_agent(id, type, size) 
{
  var g = document.createElementNS(NS,"g");
  var agent = document.createElementNS(NS,"use");
  var uid = prototypes[type] || '#{{prefix}}agent';
  agent.setAttributeNS("http://www.w3.org/1999/xlink", "xlink:href", uid);
  agent.setAttribute("transform", `scale(${size}, ${size})`)
  g.appendChild(agent)
  g.setAttribute("class", type);
  g.setAttribute("id", id);
  return g;
};

function add_entity(uid, data) {
  var id = "{{prefix}}" + uid
  if (id in entity) {
    console.warn(`Entity ${id} already present`)
    return;
  }
  var kind = data['kind'];
  var svg_world = document.getElementById("{{prefix}}world");
  var e = document.getElementById(id);
  if (e) {
    svg_world.removeChild(e);
    e = null;
  }
  var state = null;
  switch (kind) {
    case 'w':
      e = add_wall(id, data['points']);
      break;
    case 'o':
      e = add_obstacle(id, data['point'], data['radius']);
      break;
    case 'a':
      e = add_agent(id, data['type'], data['size']);
      state = data['pose']
      var [x, y, theta] = state;
      move_element(e, x, y, theta);
      agents.add(id);
      break;
    default:
      break
  }
  if (e) {
    svg_world.appendChild(e);
    entity[id] = {'svg': e, 'updated': true, 'state': state};
  }
};

function add(data) {
  for (var _id in data) {
    add_entity(_id, data[_id])
  }
  window.requestAnimationFrame(update);
};

function set_entity(_id, data) {
  if(!(_id in entity)) {
    console.warn(`Entity ${_id} unknown`)
    return;
  }
  for (a in data) {
    entity[_id]['svg'].setAttribute(a, data[a])
  }
};

function set(data) {
  for (var _id in data) {
    if (_id in entity) {
      set_entity('{{prefix}}' + _id, data[_id])
    } else {
      var e = document.getElementById("{{prefix}}" + _id);
      if (e) {
        var e_data = data[_id];
        for (a in e_data) {
          console.log(`setting attributes ${a} of ${e} to ${e_data[a]}`)
          e.setAttribute(a, e_data[a]);
        }
      }
    }
  }
  window.requestAnimationFrame(update);
};

function move_entity(_id, data) {
  if(!agents.has(_id)) {
    console.warn(`Agent ${_id} unknown`)
    return;
  }
  var [x, y, theta] = data;
  entity[_id]['state'] = [x, y, theta]
  entity[_id]['updated'] = true
};

function move(data) {
  for (var _id in data) {
    move_entity('{{prefix}}' + _id, data[_id])
  }
  window.requestAnimationFrame(update);
};

function reset() {
  var svg_world = document.getElementById("{{prefix}}world");
  for (var _id in entity) {
    svg_world.removeChild(entity[_id]['svg'])
  }
  console.log("reset")
  entity = {}
  agents.clear()
  window.requestAnimationFrame(update);
};

function view(data) {
  var [x, y, width, height] = data;
  var svg = document.getElementById("{{prefix}}svg");
  svg.setAttribute("viewBox", `${x} ${y} ${width} ${height}`)
  window.requestAnimationFrame(update);
};

function update(step) {
  var ids_to_be_removed = []
  for (var id of agents) {
    // if (!entity[id]['updated']) {
    //   ids_to_be_removed.push(id)
    // }
    if (entity[id]['updated']) {
      var [x, y, theta] = entity[id]['state']
      move_element(entity[id]['svg'], x, y, theta)
      entity[id]['updated'] = false;
    }
  }
  // for (var _id of ids_to_be_removed) {
  //   remove_entity(_id);
  // }  
};

function remove_entity(_id) {
    var svg_world = document.getElementById("{{prefix}}world");
    console.log(`remove ${_id}`)
    svg_world.removeChild(entity[_id]['svg'])
    agents.delete(_id);
    delete entity[_id];
}

function move_element(g, x, y, angle) {
  var t = `translate(${x},${y}) rotate(${angle * 180 / 3.14159})`
  g.setAttribute("transform", t)
};

