<?php
require_once('Canvas.class.php');

class DefaultCanvas extends Canvas {
	public function updateCanvas(){
		return '
			var ctx = document.querySelector("canvas").getContext("2d");
			ctx.fillStyle = "#ccc";		
			ctx.beginPath();
			ctx.rect(0,0,CANVAS_SIZE * CANVAS_SIZE_ENLARGEMENT_FACTOR,CANVAS_SIZE * CANVAS_SIZE_ENLARGEMENT_FACTOR);
			ctx.fill();
			ctx.fillStyle = "#fff";	
			
			for (var x = 1; x<CANVAS_SIZE * CANVAS_SIZE_ENLARGEMENT_FACTOR; x+= DISPLAY_SQUARE_SIZE){
				for (var y = 1; y<CANVAS_SIZE * CANVAS_SIZE_ENLARGEMENT_FACTOR; y+= DISPLAY_SQUARE_SIZE){
					ctx.beginPath();
					ctx.rect(x,y,DISPLAY_SQUARE_SIZE-1,DISPLAY_SQUARE_SIZE-1);
					ctx.fill();
				}
			}
		';
	}
	public function canvasClicked(){
		return '
			var canvas = document.getElementById("canvas");
			var rect = canvas.getBoundingClientRect();
			
			var columns = canvas.width / DISPLAY_SQUARE_SIZE;
			var rows = canvas.height / DISPLAY_SQUARE_SIZE;
			
			x = event.clientX - rect.left;
			y = event.clientY - rect.top;
			
			x -= x % DISPLAY_SQUARE_SIZE; 
			y -= y % DISPLAY_SQUARE_SIZE;
			
			var normalized_x = x / DISPLAY_SQUARE_SIZE;
			var normalized_y = y / DISPLAY_SQUARE_SIZE;
			
			console.log("x:"+x+",y:"+y);
			console.log("nx:"+normalized_x+",ny:"+normalized_y);
			console.log("rows:"+rows+",columns:"+columns);
			
			//		+y
			//		|
			//		|
			//-x ===o=== +x
			//		|
			//		|
			//		-y
			
			var ntx = (-(rows/2)) + normalized_x; //Get relative normalized target x
			var tx = SQUARE_SIZE * ntx; //To real x position
			tx += SQUARE_SIZE / 2; //Adjust to center of square
			
			var nty = (-(columns/2)) + normalized_y;//Get relative normalized target y
			var ty = -(SQUARE_SIZE * nty);//To real y position
			ty -= SQUARE_SIZE / 2;//Adjust to center of square
			
			console.log("tx:"+tx+",ty:"+ty);
			
					
			var ctx = document.querySelector("canvas").getContext("2d");
			if (clear){
				ctx.fillStyle = "#fff";
			}
			else {
				ctx.fillStyle = SQUARE_COLOR;
			}
			var target = productStepHandler.getProperty("target");
			target.move = new Array();
			target.move.z = 0;
			target.move.x = tx;
			target.move.y = ty;
			productStepHandler.setProperty("target",target);
			var key = (normalized_y * rows) + normalized_x;
			var serviceID = 10;
				if (clear){
					productStepHandler.removeProductStep(key);
				}
				else {
					productStepHandler.insertProductStep(key,serviceID);
				}	
			
			ctx.beginPath();
			ctx.rect(x + 1, y + 1, DISPLAY_SQUARE_SIZE - 1, DISPLAY_SQUARE_SIZE - 1);
			ctx.fill();
		';
	}
}

?>