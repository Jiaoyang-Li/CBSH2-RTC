var map = "" , map_h = 0 , map_w = 0 ;
var paths = [] ;
var cnt = 0 ;
function readMap(file) {
  var reader = new FileReader();
  
  reader.onload = function(e) {
    var contents = e.target.result;
    if(contents[0] == 't'){
        var lines = e.target.result.split("\n") ;
        for(let i = 0 ; i < lines[1].length ; i++){
            let currentChar = lines[1].charAt(i);
            if(!isNaN(currentChar)){
                map_h *= 10 ;
                map_h += currentChar - '0' ;
            }
        }
        for(let i = 0 ; i < lines[2].length ; i++){
            let currentChar = lines[2].charAt(i);
            if(!isNaN(currentChar)){
                map_w *= 10 ;
                map_w += currentChar - '0' ;
            }
        }
        for(let i = 4 ; i < lines.length ; i++){
            map += lines[i] ;
        }
    }else{
        for(let i = 0 ; i < contents.length ; i++){
            let currentChar = contents.charAt(i);
            if(!isNaN(currentChar)){
                map_h *= 10 ;
                map_h += currentChar - '0' ;
            }else if(currentChar == ','){
                i++ ;
                for( ; i < contents.length ; i++){
                    currentChar = contents.charAt(i);
                    if(currentChar === '\r'){
                        break ;
                    }
                    if(currentChar === '\n'){
                        break ;
                    }
                    if(!isNaN(currentChar)){
                        map_w *= 10 ;
                        map_w += currentChar - '0' ;
                    }
                }
                for(let j = i ; j < contents.length ; j++){
                    currentChar = contents.charAt(j);
                    if(currentChar === '\r' || currentChar === '\n'){
                        continue ;
                    }
                    map += (currentChar) ;
                }
                break ;
            }
        }
    }
    loadMap() ;
    
  };
  reader.readAsText(file);
}

var canvas ;
var context ;
var canvas_width , canvas_height ;
var width , height ;
// 示例用法
function loadMap(){
    canvas = document.getElementById('myCanvas');
    context = canvas.getContext('2d');
    canvas_width = canvas.width , canvas_height = canvas.height;

    width = canvas_width / map_w ;
    height = canvas_height / map_h ;
    for (let i = 0; i < map_w ; i++) {
        for(let j = 0 ; j < map_h ; j++){
            let x = j * width;
            let y = i * height;
            //console.log(map[i*map_w + j]) ;
            if(map[i*map_w + j] == '.'){
                context.fillStyle = 'grey';
            }else if(map[i*map_w + j] == '@'){
                context.fillStyle = 'black';
            }else if(map[i*map_w + j] == 'T'){
                context.fillStyle = 'olive';
            }else{
                continue ;
            }
            context.fillRect(x, y, 0.95*width, 0.95*height);
        }   
            
    } 
}
var canvas1 ;
var context1 ;

function move(x , y , state  , agent){
    if(state == 0)context1.fillStyle = "red" ;
    else context1.fillStyle = "green" ;
    context1.fillRect((y)*width, (x)*height , 0.95*width, 0.95*height);
    //context1.font = "24px Arial";
    context1.fillStyle = "white";
    context1.fillText(agent.toString(), (y)*width+width/4, (x)*height+height/2);
}
function move1(x , y){
    context1.fillStyle = "blue" ;
    context1.fillRect((y)*width, (x)*height, 0.95*width, 0.95*height);
}
function solve(j){
    context1.clearRect(0, 0, canvas1.width, canvas1.height);
    for(let i = 0 ; i < paths.length ; i ++){
        // if(i == 0){
        //     move1(paths[i][j] , paths[i][j+1]);
        // }else 
        if(j >= paths[i].length){
            move(paths[i][paths[i].length-3] , paths[i][paths[i].length-2] ,1 , i);
        }else{
            move(paths[i][j] , paths[i][j+1] , 0 , i);
        }
    }
}
function draw(){
    canvas1 = document.getElementById('myCanvas1');
    context1 = canvas1.getContext('2d');
    var max_len = 0 ;
    for(let i = 0 ; i < paths.length ; i ++){
        max_len = Math.max(max_len , paths[i].length) ;
    }
}
  
function handleFileSelect(event) {
    var file = event.target.files[0];
    var reader = new FileReader();

    reader.onload = function(event) {
        var csvData = event.target.result;
        processData(csvData);
        draw() ;
    };

    reader.readAsText(file);
}

function processData(csvData) {
    var lines = csvData.split("\n");
    for (var i = 0; i < lines.length; i++) {
        var idx = lines[i].indexOf(":") ;
        var str = lines[i].substring(idx+2) ;
        var values = str.split("->");
        var p = [] ;
        if (values.length > 1) {
            for(let j = 0 ; j < values.length ; j++){
                var x = 0 ;
                for(let k = 1 ; k < values[j].length-1 ; k ++){
                    if(values[j][k] === ','){
                        p.push(x) ;
                        x = 0 ;
                        continue ;
                    }
                    x *= 10 ;
                    x += values[j][k] - '0' ;
                }
                p.push(x) ;
            }
          paths.push(p) ;
        }
    }
}

document.addEventListener('DOMContentLoaded', function() {
    // 在这里放置你的 JavaScript 代码
    var fileInput = document.getElementById('file-input'); // 获取文件输入框元素
    fileInput.addEventListener('change', function(e) {
        var file = e.target.files[0]; // 获取选择的文件
        readMap(file); // 调用函数读取文件内容
    });
    var csvInput = document.getElementById('csvFileInput') ;
    csvInput.addEventListener('change' , function(e) {
        handleFileSelect(e) ;
    });
    var button = document.getElementById("myButton") ;
    
    button.addEventListener("click", function(){
        solve(cnt) ;
        cnt += 2 ;
    });
  });