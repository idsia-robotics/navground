var NS = "http://www.w3.org/2000/svg";

class WebUIClient {
  constructor(port='8000', prefix='', display_shape=false) {
    this.agents = new Set();
    this.entity = {};
    this.prefix = prefix;
    this.port = port;
    this.display_shape = display_shape;
    this.prototypes = {
      '': `${prefix}agent`, 
      'thymio': `${prefix}thymio`, 
      'wheelchair': `${prefix}wheelchair`, 
      'human': `${prefix}human`
    };
  }

  connect() {
    var client = this;
    var ws_address = `ws://127.0.0.1:${this.port}/`
    console.log(`[${this.prefix}] Try to connect to ${ws_address}`)
    var ws = new WebSocket(ws_address);
    ws.onopen = function(event) {
      console.log(`[${client.prefix}] Opened Web socket ${ws_address}`)
      client.ws = ws;
    };
    
    ws.onmessage = function(event) {
      var [type, data] = JSON.parse(event.data)
      switch (type) {
        case 'm':
          client.move(data)
          break;
        case 'r':
          client.reset(data)
          break;
        case 's':
          client.set(data)
          break;
        case 'v':
          client.view(data)
          break;
        case '+':
          client.add(data)
          break;
        default:
          break
      }
    }
  
    ws.onclose = function(e) {
      console.log('Socket is closed. Reconnect will be attempted in 1 second.', e.reason);
      client.ws = null;
      setTimeout(function(){client.connect();}, 1000);
    };
  
    ws.onerror = function(err) {
      console.log('Socket encountered error ... closing socket');
      ws.close();
    };
  }

  add_wall(id, points) {
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
  
  add_obstacle(id, point, radius) {
    var e = document.createElementNS(NS,"circle");
    e.setAttribute("cx", `${point[0]}`)
    e.setAttribute("cy", `${point[1]}`)
    e.setAttribute("r", `${radius}`)
    e.setAttribute("class", "obstacle");
    e.setAttribute("id", id);
    return e
  }
  
  add_agent(id, type, size, color) 
  {
    var g = document.createElementNS(NS,"g");
    var agent = document.createElementNS(NS,"use");
    var uid = this.prototypes[type] || `${this.prefix}agent`;
    agent.setAttributeNS("http://www.w3.org/1999/xlink", "xlink:href", "#" + uid);
    agent.setAttribute("transform", `scale(${size}, ${size})`)
    g.appendChild(agent)
    g.setAttribute("class", type);
    g.setAttribute("id", id);
    if (color) {
      g.setAttribute("style", `fill: ${color}`);
    }
    if (this.display_shape) {
      var shape = document.createElementNS(NS,"circle");
      shape.setAttribute("r", size);
      shape.setAttribute("class", "shape");
      g.appendChild(shape);
    }
    return g;
  };
  
  add_entity(uid, data) {
    var id = this.prefix + uid
    if (id in this.entity) {
      console.warn(`Entity ${id} already present`)
      return;
    }
    var kind = data['kind'];
    var svg_world = document.getElementById(this.prefix + "world");
    if (!svg_world) return;
    var e = document.getElementById(id);
    if (e) {
      svg_world.removeChild(e);
      e = null;
    }
    var state = null;
    switch (kind) {
      case 'w':
        e = this.add_wall(id, data['points']);
        break;
      case 'o':
        e = this.add_obstacle(id, data['point'], data['radius']);
        break;
      case 'a':
        e = this.add_agent(id, data['type'], data['size'], data['color']);
        state = data['pose']
        var [x, y, theta] = state;
        this.move_element(e, x, y, theta);
        this.agents.add(id);
        break;
      default:
        break
    }
    if (e) {
      svg_world.appendChild(e);
      this.entity[id] = {'svg': e, 'updated': true, 'state': state};
    }
  };
  
  add(data) {
    for (var _id in data) {
      this.add_entity(_id, data[_id])
    }
    window.requestAnimationFrame(this.update.bind(this));
  };
  
  set_entity(_id, data) {
    if(!(_id in this.entity)) {
      console.warn(`Entity ${_id} unknown`)
      return;
    }
    for (var a in data) {
      this.entity[_id]['svg'].setAttribute(a, data[a])
    }
  };
  
  set(data) {
    for (var _id in data) {
      var _nid = this.prefix + _id
      if (_nid in this.entity) {
        this.set_entity(_nid, data[_id])
      } else {
        var e = document.getElementById(_nid);
        if (e) {
          var e_data = data[_id];
          for (var a in e_data) {
            console.log(`Setting attributes ${a} of ${e} to ${e_data[a]}`)
            e.setAttribute(a, e_data[a]);
          }
        }
      }
    }
    window.requestAnimationFrame(this.update.bind(this));
  };
  
  move_entity(_id, data) {
    if(!this.agents.has(_id)) {
      console.warn(`Agent ${_id} unknown`)
      return;
    }
    var [x, y, theta] = data;
    this.entity[_id]['state'] = [x, y, theta]
    this.entity[_id]['updated'] = true
  };
  
  move(data) {
    for (var _id in data) {
      this.move_entity(this.prefix + _id, data[_id])
    }
    window.requestAnimationFrame(this.update.bind(this));
  };
  
  reset() {
    var svg_world = document.getElementById(this.prefix + "world");
    if (!svg_world) return;
    for (var _id in this.entity) {
      svg_world.removeChild(this.entity[_id]['svg'])
    }
    console.log(`[${this.prefix}] reset`)
    this.entity = {}
    this.agents.clear()
    window.requestAnimationFrame(this.update.bind(this));
  };
  
  view(data) {
    var [x, y, width, height] = data;
    var svg = document.getElementById(this.prefix + "svg");
    if (!svg) return;
    svg.setAttribute("viewBox", `${x} ${y} ${width} ${height}`)
    window.requestAnimationFrame(this.update.bind(this));
  };
  
  update(step) {
    var ids_to_be_removed = []
    for (var id of this.agents) {
      // if (!entity[id]['updated']) {
      //   ids_to_be_removed.push(id)
      // }
      if (this.entity[id]['updated']) {
        var [x, y, theta] = this.entity[id]['state']
        this.move_element(this.entity[id]['svg'], x, y, theta)
        this.entity[id]['updated'] = false;
      }
    }
    // for (var _id of ids_to_be_removed) {
    //   remove_entity(_id);
    // }  
  };
  
  remove_entity(_id) {
      var svg_world = document.getElementById(this.prefix + "world");
      if (!svg_world) return;
      console.log(`remove ${_id}`)
      svg_world.removeChild(this.entity[_id]['svg'])
      this.agents.delete(_id);
      delete this.entity[_id];
  }
  
  move_element(g, x, y, angle) {
    var t = `translate(${x},${y}) rotate(${angle * 180 / 3.14159})`
    g.setAttribute("transform", t)
  };
}

