'use strict';
var socket = io();
const img = document.getElementById("map");
var originX = 0;
var originY = 0;
var mapRes = 0;
const robot = document.getElementById("robot")
const control = document.getElementById("control")
const horz = document.getElementById("horz")
const ctrlpn = document.getElementById("controlpanel")

window.ondragstart = ()=>false;

socket.on('connect', ()=>{
    socket.on("map",(b)=>{
        img.style.backgroundImage = `url(data:image/png;base64,${b.map})`;
        let wh = new Image();
        wh.onload=()=>{
            img.style.width = `${wh.width}px`;
            img.style.height = `${wh.height}px`;
        }
        wh.src = `data:image/png;base64,${b.map}`
        originX = parseFloat(b.x);
        originY = parseFloat(b.y);
        mapRes = parseFloat(b.r);
        console.log("map");
    })
    socket.on("pos",(b)=>{
        // convert to pixels
        robot.style.top = `${(parseFloat(b.x)-originX)/mapRes}px`
        robot.style.left = `${(parseFloat(b.y)-originY)/mapRes}px`
        robot.style.transform = `rotate(${-parseFloat(b.r)}rad) translate(-50%,-50%)`
    })
});

function element_offsets(e) {
    var left = 0, top = 0;
    do {
        left += e.offsetLeft;
        top += e.offsetTop;
    } while (e = e.offsetParent);
    return { left: left, top: top };
}

var lastclick;

img.addEventListener("mousedown",(e)=>{
    e.stopPropagation();
    console.log("down");
    lastclick = e;
})
let f = (r) => {
    console.log("up")
    let ofs = element_offsets(img);
    let x = lastclick.pageX - ofs.left;
    let y = lastclick.pageY - ofs.top;
    let xd = r.pageX - lastclick.pageX;
    let yd = r.pageY - lastclick.pageY;
    let angle = Math.atan2(xd, yd);
    console.log(x,mapRes,originX);
    console.log(x*mapRes+originX,"e", y*mapRes+originY,"e", angle);
    socket.emit("goal", { x: x * mapRes + originX, y: y * mapRes + originY, z: 1.0, ox: 0.0, oy: 0.0, oz: Math.sin(angle / 2), ow: Math.cos(angle / 2) })
}
img.addEventListener("mouseup",f);

var ismousedown = false;

control.addEventListener("mousedown",(r)=>{
    ismousedown = true;
    updatespeed(r);
})
control.addEventListener("mouseup",()=>{
    if (ismousedown){
        socket.emit("vel", { "f": 0, "h": 0, "r": 0 });
    }
    ismousedown = false;
})

function updatespeed(r){
    let ofs = element_offsets(control);
    let x = r.pageX - ofs.left - 200;
    let y = r.pageY - ofs.top - 200;
    console.log(y/200,x/200);
    if (horz.checked){
        socket.emit("vel", { "f": -y/200, "h": -x/200, "r": 0});
    } else{
        socket.emit("vel", { "f": -y/200, "h": 0, "r": -x/100});
    }
}

control.addEventListener("mousemove",(r)=>{
    if (ismousedown){
        updatespeed(r);
    }
})

var sF = 0;
var sH = 0;
var sR = 0;
ctrlpn.addEventListener("click",(e)=>{
    if (e.target.getAttribute("data-s")=='!'){
        sF = 0;
        sH = 0;
        sR = 0;
        socket.emit("vel", { "f": 0, "h": 0, "r": 0 });
        return;
    }
    let sp = e.target.getAttribute("data-s").split(",");
    console.log(sp);
    sF+=parseFloat(sp[0]);
    sH+=parseFloat(sp[1]);
    sR+=parseFloat(sp[2]);
    socket.emit("vel", { "f": sF, "h": sH, "r": sR});
})