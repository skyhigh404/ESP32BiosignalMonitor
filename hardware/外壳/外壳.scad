
$fa = 1;
$fs = 0.4;

// 考虑了0.2mm公差，x和y方向尺寸
width = 87.8;
height = 34.7; 
// 倒角直径
diameter = 6;

//底座打印厚度
thickness = 2;

//边缘厚度
edge_thick = 2;

//绑带厚度
tape_thick = 1.8;

//绑带宽度
tape_width = 22;

//电池厚度10mm，绑带厚度
battery_thick = 10.6+tape_thick;



//顶部高度
top_high = 11;

color("red")
linear_extrude(height = thickness){
    //做倒角
    minkowski(){
        square([width-diameter,height-diameter],center = true);
    circle(d = diameter);
}
}



// 四分之一圆环
module quater_ring(large_diameter, small_diameter){
intersection(){
difference(){
circle(d=large_diameter);
circle(d=small_diameter);
}
square(large_diameter);    
}
}

// 外边界
// 右上角
translate([width/2-diameter/2-0.001,height/2-diameter/2-0.001,0])
linear_extrude(height = battery_thick+thickness+top_high){
quater_ring(diameter+2*edge_thick,diameter);
}

// 左上角
translate([-width/2+diameter/2+0.001,height/2-diameter/2-0.001,0])
linear_extrude(height = battery_thick+thickness+top_high){
rotate([0,0,90])
quater_ring(diameter+2*edge_thick,diameter);
}

// 左下角
translate([-width/2+diameter/2+0.001,-height/2+diameter/2+0.001,0])
linear_extrude(height = battery_thick+thickness){
rotate([0,0,180])
quater_ring(diameter+2*edge_thick,diameter);
}


// 右下角
translate([width/2-diameter/2-0.001,-height/2+diameter/2+0.001,0])
linear_extrude(height = battery_thick+thickness){
rotate([0,0,270])
quater_ring(diameter+2*edge_thick,diameter);
}

// 左挡板，并托住电路板
translate([-width/2-edge_thick+0.001,-height/2+diameter/2,0])
cube([edge_thick+2,height-diameter,battery_thick+thickness]);

// 右挡板，并托住电路板
translate([width/2-2-0.001,-height/2+diameter/2,0])
cube([edge_thick+2,height-diameter,battery_thick+thickness]);

// 前挡板，绑带孔
difference(){
translate([-(width-diameter)/2,-height/2-edge_thick+0.001,0])
cube([width-diameter,edge_thick,battery_thick+thickness+top_high]);
translate([0,0,tape_thick/2+thickness+0.2])
cube([tape_width,50,tape_thick],center = true);
}
// 后挡板，留出电池线通道，绑带孔
difference(){
translate([-(width-diameter)/2+18,height/2-0.001,0])
cube([width-diameter-18,edge_thick,battery_thick+thickness+top_high]);
translate([0,0,tape_thick/2+thickness+0.2])
cube([tape_width,50,tape_thick],center = true);
}











