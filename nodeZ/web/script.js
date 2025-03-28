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
const goal = document.getElementById("goal")

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
    socket.on("ping",()=>{
        console.log("received ping event")
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
var lcX = -1;
var lcY;

img.addEventListener("mousedown",(e)=>{
    e.stopPropagation();
    console.log("down");
    let ofs = element_offsets(img);
    lcX = e.pageX - ofs.left;
    lcY = e.pageY - ofs.top;
    lastclick = e;
    goal.hidden = false;
    goal.style.left = `${lcX}px`;
    goal.style.top = `${lcY}px`;
    goal.style.transform = `translate(-50%, -100%)`
})

addEventListener("mousemove",(r)=>{
    if (lcX != -1) {
        let xd = r.pageX - lastclick.pageX;
        let yd = r.pageY - lastclick.pageY;
        let angle = Math.atan2(xd, yd);
        goal.style.transform = `translate(-50%, -100%) rotate(${-angle + Math.PI}rad)`
    }
})

let f = (r) => {
    console.log("up")
    let xd = r.pageX - lastclick.pageX;
    let yd = r.pageY - lastclick.pageY;
    let angle = Math.atan2(xd, yd);
    goal.style.transform = `translate(-50%, -100%) rotate(${-angle+Math.PI}rad)`
    socket.emit("goal", { x: lcX * mapRes + originX, y: lcY * mapRes + originY, z: 1.0, ox: 0.0, oy: 0.0, oz: Math.sin(angle / 2), ow: Math.cos(angle / 2) })
    lcX = -1
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