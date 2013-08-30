function Click() {
	/*
	 * Public functions
	 */

	this.init = function() {
		var is_touch_device = 'ontouchstart' in document.documentElement;
		if (is_touch_device) {
			initTouchDevice();
		} else {
			initNonTouchDevice();
		}
	}
	/*
	 * Private functions
	 */
	var initTouchDevice = function() {
		drawer = {
			touchstart : function(coors) {
				DownEventHandler(coors);
			},
			touchmove : function(coors) {
				MoveEventHandler(coors);
			},
			touchend : function(coors) {
				UpEventHandler(coors);
			}
		};

		document.addEventListener('touchstart', touchDrawHandler, false);
		document.addEventListener('touchmove', touchDrawHandler, false);
		document.addEventListener('touchend', touchDrawHandler, false);

		document.addEventListener('touchmove', function(event) {
			event.preventDefault();
		}, false);
	}
	var initNonTouchDevice = function() {
		document.addEventListener("mousedown", DownEventHandler, false);
		document.addEventListener("mouseup", UpEventHandler, false);
		document.addEventListener("mousemove", MoveEventHandler, false);
	}
	var touchDrawHandler = function(event) {
		if (event.type == 'touchend') {
			var coors = {
				x : event.changedTouches[0].pageX,
				y : event.changedTouches[0].pageY,
				clientX : event.changedTouches[0].pageX,
				clientY : event.changedTouches[0].pageY
			};
		} else {
			var coors = {
				x : event.targetTouches[0].pageX,
				y : event.targetTouches[0].pageY,
				clientX : event.targetTouches[0].pageX,
				clientY : event.targetTouches[0].pageY
			};
		}

		var obj = window;

		if (obj.offsetParent) {
			do {
				coors.x -= obj.offsetLeft;
				coors.y -= obj.offsetTop;
				coors.clientX -= obj.offsetLeft;
				coors.clientY -= obj.offsetTop;
			} while ((obj = obj.offsetParent) != null);
		}

		// pass the coordinates to the appropriate handler
		drawer[event.type](coors);
	}
}
