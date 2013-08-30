
productList = [
               {"id":1, "name":"one", "status":"Completed", "started": "2013-03-01 10:30", "completed": "2013-03-01 10:30", "progress":100},
               {"id":2, "name":"two", "status":"Completed", "started": "2013-03-01 10:30", "completed": "2013-03-01 10:30", "progress":100},
               {"id":3, "name":"three", "status":"Processing", "started": "2013-03-01 10:30", "completed": "", "progress":89},
               {"id":4, "name":"six", "status":"Waiting", "started": "", "completed": "", "progress":0}
               ];


function refresh(){
	console.log('refresh');
	showSwirl(true);
	var table=document.getElementById("productOverviewTable");
	
	for(var i = table.rows.length - 1; i > 0; i--)
	{
	    table.deleteRow(i);
	}
	
	for(val in productList){
		
		var row=table.insertRow(-1);
		row.id = "Row" + val;
		
		row.onclick=function(){
		console.log(this.id); 
		$("#testLabel").html( productList[this.id.replace("Row","")].name );
		$(".pop").slideFadeToggle(function() { 
            $("#email").focus();
        });};
		var cell1=row.insertCell(0);
		var cell2=row.insertCell(1);
		var cell3=row.insertCell(2);
		var cell4=row.insertCell(3);
		var cell5=row.insertCell(4);
		cell1.innerHTML=productList[val].name;
		
		cell2.innerHTML=productList[val].status;
		cell3.innerHTML=productList[val].started;
		cell4.innerHTML=productList[val].completed;
		cell5.innerHTML="<canvas id='productProgressBar" + productList[val].id + "' class='processCanvas' height='15' width='200'> No support for HTML5 Canvas.</canvas>";
		
		
		setProgress("productProgressBar"+productList[val].id , productList[val].progress);
	}
	showSwirl(false);
	
}

function setProgress(canvasID, percentage){
	

perc=percentage*2;
var canvas = document.getElementById(canvasID);
var x = canvas.width / 2;
var y = canvas.height/4*3;


if (canvas.getContext) {
	var ctx = canvas.getContext("2d");
	ctx.clearRect(0, 0, canvas.width, canvas.height)
	ctx.fillStyle = "rgb(150,150,255)";
	ctx.fillRect (0, 0, perc, canvas.height);
	
	ctx.fillStyle = "rgb(0,0,0)";
	ctx.font=canvas.height*0.75 + "px Arial";
	ctx.textAlign = 'center';
	ctx.fillText(percentage + "%", x, y);
	
}
}

function showSwirl(show){
	if(show)
		document.getElementById('swirl').style.display = 'block';
	else
		document.getElementById('swirl').style.display = 'none';
	
	//$('#swirl').display = 'block'; 
}

function deselect() {
    $(".pop").slideFadeToggle(function() { 
        $("#contact").removeClass("selected");
    });    
}

$(function() {
    $("#contact").click(function() {
        if($(this).hasClass("selected")) {
            deselect();               
        } else {
            $(this).addClass("selected");
            $(".pop").slideFadeToggle(function() { 
                $("#email").focus();
            });
        }
        return false;
    });

    $(".close").click( function() {
        deselect();
        return false;
    });
});

$.fn.slideFadeToggle = function(easing, callback) {
    return this.animate({ opacity: 'toggle', height: 'toggle' }, "fast", easing, callback);
};
