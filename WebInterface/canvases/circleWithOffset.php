<?php
require_once('Canvas.class.php');

class CirclewithoffsetCanvas extends Canvas {
	public function updateCanvas(){
		return '
			var DISPLAY_SQUARE_SIZE = SQUARE_SIZE * CANVAS_SIZE_ENLARGEMENT_FACTOR;
			var DISPLAY_SQUARE_OFFSET = SQUARE_OFFSET * CANVAS_SIZE_ENLARGEMENT_FACTOR;
			var DISPLAY_SIZE = (CANVAS_SIZE * (DISPLAY_SQUARE_SIZE + DISPLAY_SQUARE_OFFSET)) + DISPLAY_SQUARE_OFFSET;
			document.getElementById("canvas").width = DISPLAY_SIZE;
			document.getElementById("canvas").height = DISPLAY_SIZE;
			var ctx = document.querySelector("canvas").getContext("2d");
			ctx.fillStyle = "#ccc";		
			ctx.beginPath();
			ctx.rect(0,0,DISPLAY_SIZE,DISPLAY_SIZE);
			ctx.fill();
			ctx.fillStyle = "#fff";	
			
			for (var x = DISPLAY_SQUARE_OFFSET + (DISPLAY_SQUARE_SIZE/2); x<DISPLAY_SIZE; x+=DISPLAY_SQUARE_SIZE + DISPLAY_SQUARE_OFFSET){
				for (var y = DISPLAY_SQUARE_OFFSET + (DISPLAY_SQUARE_SIZE/2); y<DISPLAY_SIZE; y+=DISPLAY_SQUARE_SIZE + DISPLAY_SQUARE_OFFSET){
					ctx.beginPath();
					ctx.arc(x,y,((DISPLAY_SQUARE_SIZE)/2)-1,0,2*Math.PI);
					ctx.fill();
				}
			}
		';
	}
	public function canvasClicked(){
		return '
			var DISPLAY_SQUARE_OFFSET = SQUARE_OFFSET * CANVAS_SIZE_ENLARGEMENT_FACTOR;
			console.log("test");
			var canvas = document.getElementById("canvas");
			var rect = canvas.getBoundingClientRect();
			
			var columns = CANVAS_SIZE;
			var rows = CANVAS_SIZE;
			
			x = event.clientX - rect.left;
			y = event.clientY - rect.top;
			
			x -= x % (DISPLAY_SQUARE_SIZE + DISPLAY_SQUARE_OFFSET); 
			y -= y % (DISPLAY_SQUARE_SIZE + DISPLAY_SQUARE_OFFSET);
			
			var normalized_x = x / (DISPLAY_SQUARE_SIZE + DISPLAY_SQUARE_OFFSET);
			var normalized_y = y / (DISPLAY_SQUARE_SIZE + DISPLAY_SQUARE_OFFSET);
			
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
			var tx = (SQUARE_SIZE) * ntx; //To real x position
			console.log("ntx:"+ntx);
			if (ntx > -1){
				tx += SQUARE_OFFSET / 2;
				tx += SQUARE_OFFSET * ntx;
			}
			else {
				tx -= SQUARE_OFFSET / 2;
				tx -= SQUARE_OFFSET * (-(ntx+1));
			}
			tx += SQUARE_SIZE / 2; //Adjust to center of square
			
			var nty = (-(columns/2)) + normalized_y;//Get relative normalized target y
			var ty = -(SQUARE_SIZE * nty);//To real y position
			if (nty > -1){
				ty -= SQUARE_OFFSET / 2;
				ty -= SQUARE_OFFSET * nty;
			}
			else {
				ty += SQUARE_OFFSET / 2;
				ty += SQUARE_OFFSET * (-(nty+1));
			}
			ty -= SQUARE_SIZE / 2;//Adjust to center of square
			
			
					
			var ctx = document.querySelector("canvas").getContext("2d");
			if (clear){
				ctx.fillStyle = "#fff";
			}
			else {
				ctx.fillStyle = SQUARE_COLOR;
			}
			var target = productStepHandler.getProperty("target");
			target.move = {};
			target.move.x = tx;
			target.move.y = ty;
			target.move.z = -26.5;
			target.move.approach = {};
			target.move.approach.x = 0;
			target.move.approach.y = 0;
			target.move.approach.z = 20;
			console.log("tx:"+(tx)+",ty:"+ty);		
			
			productStepHandler.setProperty("target",target);
			var key = (normalized_y * rows) + normalized_x;
			var serviceID = 20;
			
				if (clear){
					productStepHandler.removeProductStep(key);
				}
				else {
					productStepHandler.insertProductStep(key,serviceID);
				}	
			
			ctx.beginPath();
			ctx.arc(x + ((DISPLAY_SQUARE_SIZE)/2) + DISPLAY_SQUARE_OFFSET, y + (DISPLAY_SQUARE_SIZE/2) + DISPLAY_SQUARE_OFFSET, DISPLAY_SQUARE_SIZE/2, 0, 2*Math.PI);
			ctx.fill();
		';
	}
}

?>
