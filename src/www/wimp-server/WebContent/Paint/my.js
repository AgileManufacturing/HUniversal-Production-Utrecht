/*
 * Global variables
 */

var canvas;
var canvasContext;
var rows = parent.document.getElementById('paintAppHeight').value;
var columns = parent.document.getElementById('paintAppWidth').value;

var canvasMarginPercentage = 90;
var canvasBorder = 3;
var mouseDown = false;
var pixels = new Array(rows * columns);
var pixelColor = "#0000FF";
var backgroundPixelColor = "#FFFFFF";
var menuClosed = true;
var menuTickSpeed = 150;
var menuIsMoving = false;
var menuLeft;

//for algorithms
var blue = new color(0, 0, 255, 255);
var yellow = new color(255, 255, 0, 255);
var red = new color(255, 0, 0, 255);
var white = new color(255, 255, 255, 255);
var colors = [red, yellow, blue, white];
var pixelColorArray = new Array();

/*
 * Init functions
 */

function initWindow() {
	initSpinner();

	canvas = document.getElementById("drawingCanvas");
	canvasContext = canvas.getContext("2d");

	initCanvasSize();

	var uploadImage = document.getElementById('imageLoader');
	uploadImage.addEventListener('change', imageLoadedEventHandler, false);

	window.parent.showCreateTab('paint');
	window.parent.showNotification('information', 'Press left on the arrow to show and hide the color menu :)', 3000);
}

function initCanvasSize() {
	var canvasPossibleWidth = (getWidth() / 100) * canvasMarginPercentage;
	var canvasPossibleHeight = (getHeight() / 100) * canvasMarginPercentage;
	canvasPossibleWidth -= canvasBorder * 2;
	canvasPossibleHeight -= canvasBorder * 2;

	var i1 = canvasPossibleWidth / columns;
	var i2 = canvasPossibleHeight / rows;

	if (i1 > i2) {
		itemSize = i2;
	} else {
		itemSize = i1;
	}

	canvasContext.canvas.width = itemSize * columns;
	canvasContext.canvas.height = itemSize * rows;

	document.getElementById("topOffset").style.height = (getHeight() / 2) - (canvas.height / 2);
	drawAllCanvasItems();
}

function initSpinner() {
	var target = document.getElementById('spinner');
	target.style.visibility = 'hidden';
	var spinner = new Spinner().spin();
	target.appendChild(spinner.el);
}

/*
 * Event handlers
 */

function imageLoadedEventHandler(input) {
	var reader = new FileReader();

	reader.onload = function(e) {
		showSpinner();
		console.log("Loading image..");
		var img = new Image();
		img.src = e.target.result;

		img.onload = function() {
			hideSpinner();
			console.log("Loading image done..");
			adjustImage(img, e);
		}
	}

	reader.readAsDataURL(input.target.files[0]);
}

function UpEventHandler(event) {
	mouseDown = false;
}

function MoveEventHandler(event) {
	if (mouseDown) {
		var array = getSelectedRowAndColumn(event);
		pixels[rowAndColumnToArrayNumber(array[0], array[1])] = pixelColor;
		drawPixel(array[0], array[1], pixelColor);
	}
}

function DownEventHandler(event) {
	mouseDown = true;
	var array = getSelectedRowAndColumn(event);
	pixels[rowAndColumnToArrayNumber(array[0], array[1])] = pixelColor;
	drawPixel(array[0], array[1], pixelColor);
}

/*
 * Spinner functions
 */

function showSpinner() {
	document.getElementById('spinner').style.visibility = 'visible';
}

function hideSpinner() {
	document.getElementById('spinner').style.visibility = 'hidden';
}

/*
 * Row and column functions
 */

function getSelectedRowAndColumn(event) {
	if (isInElement(canvas, event)) {
		var clickedX = event.clientX - canvas.offsetLeft;
		var clickedY = event.clientY - canvas.offsetTop;
		var column = Math.floor(clickedX / itemSize)
		var row = Math.floor(clickedY / itemSize);
		return [row, column];
	} else {
		return 'undefined';
	}
}

function rowAndColumnToArrayNumber(givenRow, givenColumn) {
	var givenRowNormal = givenRow + 1;
	var givenColumnNormal = givenColumn + 1;
	return (columns * (givenRowNormal - 1)) + (givenColumnNormal - 1);
}

function arrayNumberToRow(number) {
	return Math.floor(number / columns);
}

function arrayNumberToColumn(number) {
	return number % columns;
}

/*
 * Drawing functions
 */

function drawPixel(row, column, color) {
	if (row >= 0 && column >= 0) {
		canvasContext.fillStyle = color;

		var x1 = Math.ceil(itemSize * column);
		var y1 = Math.ceil(itemSize * row);
		var x2 = Math.ceil(itemSize);
		var y2 = x2;

		canvasContext.fillRect(x1, y1, x2, y2);
	}
}

function drawAllCanvasItems() {
	for (var i = 0; i < pixels.length; i++) {
		if (pixels[i] == undefined) {
			drawPixel(arrayNumberToRow(i), arrayNumberToColumn(i), backgroundPixelColor);
		} else {
			drawPixel(arrayNumberToRow(i), arrayNumberToColumn(i), pixels[i]);
		}
	}
}

/*
 * Menu functions
 */

function toggleMenu() {
	if (menuClosed) {
		openMenu();
	} else {
		closeMenu();
	}
}

function openMenu() {
	var menu = document.getElementById("menu");
	var stopPosition = 0;
	if (!menuIsMoving) {
		menuIsMoving = true;
		menuLeft = menu.offsetLeft;
	}

	if (parseInt(menuLeft) < stopPosition) {
		menuLeft += getMenuSpeed();
		menu.style.left = menuLeft;
		setTimeout(openMenu, 1);
	} else {
		menuIsMoving = false;
		menuClosed = false;
	}
}

function closeMenu() {
	var menu = document.getElementById("menu");
	var stopPosition = -menu.offsetWidth;
	if (!menuIsMoving) {
		menuIsMoving = true;
		menuLeft = menu.offsetLeft;
	}

	if (parseInt(menuLeft) > stopPosition) {
		menuLeft -= getMenuSpeed();
		menu.style.left = menuLeft;
		setTimeout(closeMenu, 1);
	} else {
		menuIsMoving = false;
		menuClosed = true;
	}
}

function getMenuSpeed() {
	return getWidth() / menuTickSpeed;

}

/*
 * Algorithms
 */

function adjustImage(img, event) {
	showSpinner();
	console.log("Adjusting image started..");

	//doe hier iets
	var tempCanvas = document.createElement('canvas');
	var tempCanvasContext = tempCanvas.getContext('2d');

	console.log("Drawing image on canvas");
	tempCanvas.height = img.height;
	tempCanvas.width = img.width;
	tempCanvasContext.drawImage(img, 0, 0);
	var imageData = tempCanvasContext.getImageData(0, 0, img.width, img.height);

	//Possible to use, makes it much faster but image quality will be less..
	var is_touch_device = 'ontouchstart' in document.documentElement;
	if (is_touch_device) {
		console.log("Touch device, make image smaller");
		var newWidth = img.width / 5;
		var newHeight = img.height / 5;
		tempCanvasContext.drawImage(img, 0, 0, newWidth, newHeight);
		imageData = tempCanvasContext.getImageData(0, 0, newWidth, newHeight);
	}

	console.log("Use color filter");
	var imageDataFilter = filterColor(imageData);
	// canvasContext.putImageData(imageDataFilter, 0, 0);

	console.log("Use smart resize and draw");
	resizeAndDraw(imageDataFilter);

	//klaar

	hideSpinner();
	console.log("Adjusting image finished..");
}

function filterColor(givenImageData) {
	var imageData;
	if (givenImageData == undefined) {
		imageData = canvasContext.getImageData(0, 0, canvas.width, canvas.height);
	} else {
		imageData = givenImageData;
	}

	var pixelIndex = 0;
	var k = 0;
	var numBytes = imageData.data.length;
	var otDif = 900;
	var j = 0;

	while (pixelIndex < numBytes) {
		var red = imageData.data[pixelIndex];
		var green = imageData.data[pixelIndex + 1];
		var blue = imageData.data[pixelIndex + 2];
		var alpha = imageData.data[pixelIndex + 3];

		for ( i = 0; i < colors.length; i++) {
			if (red > colors[i].R) {
				var rDif = red - colors[i].R;
			} else {
				rDif = colors[i].R - red;
			}
			if (green > colors[i].G) {
				var gDif = green - colors[i].G;
			} else {
				gDif = colors[i].G - green;
			}
			if (blue > colors[i].B) {
				var bDif = blue - colors[i].B;
			} else {
				bDif = colors[i].B - blue;
			}
			if (alpha > colors[i].A) {
				var aDif = alpha - colors[i].A;
			} else {
				aDif = colors[i].A - alpha;
			}

			var tDif = bDif + gDif + rDif + aDif;
			if (i == 3) {
				tDif = tDif * 5;
			}
			if (tDif < otDif) {
				otDif = tDif;
				j = i;
			}
		}
		otDif = 1104;

		imageData.data[pixelIndex] = colors[j].R;
		imageData.data[pixelIndex + 1] = colors[j].G;
		imageData.data[pixelIndex + 2] = colors[j].B;
		imageData.data[pixelIndex + 3] = colors[j].A;

		pixelColorArray[(pixelIndex / 4)] = j;
		pixelIndex = pixelIndex + 4;
	}

	// var canvasX = 0;
	// var canvasY = 0;

	return imageData;
}

function resizeAndDraw(imageData) {

	var newW = columns;
	var newH = rows;

	var imageDataResize = canvasContext.createImageData(newW, newH);

	var OriH = imageData.height;
	var OriW = imageData.width;
	var TempH;
	var TempW;
	var MOH = OriH % newH;
	var MOW = OriW % newW;
	var pixHeight = (OriH - MOH) / newH;
	var pixWidth = (OriW - MOW) / newW;
	var firstH = 0;
	var firstW = 0;
	var amountColor = new Array();

	for ( p = 0; p < colors.length; p++) {
		amountColor[p] = 0;
	}

	for ( l = 0; l < newH; l++) {
		if (l < (MOH)) {
			jloop = pixHeight + 1;
			if (l != 0) {
				TempH = TempH + pixHeight + 1 + firstH;
				firstH = 0;
			} else {
				TempH = 0;
				firstH = 1;
			}
		} else {
			jloop = pixHeight;
			if (l != 0) {
				TempH = TempH + pixHeight + firstH;
				firstH = 0;
			} else {
				TempH = 0;
			}
		}
		for ( k = 0; k < newW; k++) {
			if (k < (MOW)) {
				iloop = pixWidth + 1;
				if (k != 0) {
					TempW = TempW + pixWidth + 1 + firstW;
					firstW = 0;
				} else {
					TempW = 0;
					firstW = 1;
				}
			} else {
				iloop = pixWidth;
				if (k != 0) {
					TempW = TempW + pixWidth + firstW;
					firstW = 0;
				} else {
					TempW = 0;
				}
			}
			for ( j = 0; j < jloop; j++) {
				for ( i = 0; i < iloop; i++) {
					CurrPix = i + TempW + (TempH * OriW) + (j * OriW);
					amountColor[pixelColorArray[CurrPix]] = amountColor[pixelColorArray[CurrPix]] + 1;
				}
			}
			var colormost;
			var TempHighest = 0;
			for ( u = 0; u < colors.length; u++) {
				if (amountColor[u] > TempHighest) {
					TempHighest = amountColor[u];
					colormost = u;
				}
				amountColor[u] = 0;
			}

			currPixNew = (k + (l * newW)) * 4;

			imageDataResize.data[currPixNew] = colors[colormost].R;
			// red   color
			imageDataResize.data[currPixNew + 1] = colors[colormost].G;
			// green color
			imageDataResize.data[currPixNew + 2] = colors[colormost].B;
			// blue  color
			imageDataResize.data[currPixNew + 3] = colors[colormost].A;

			var column = (currPixNew / 4) % columns;
			var row = ((currPixNew / 4) - ((currPixNew / 4) % columns)) / columns;

			var color = RGB2HTML(colors[colormost].R, colors[colormost].G, colors[colormost].B);
			pixels[rowAndColumnToArrayNumber(row, column)] = color;
			drawPixel(row, column, color);
		}
	}

	return imageDataResize;
}

/*
 * Other functions
 */

function isInElement(element, event) {
	var x = event.clientX;
	var y = event.clientY;

	var elementX1 = element.offsetLeft;
	var elementY1 = element.offsetTop;
	var elementX2 = elementX1 + element.offsetWidth;
	var elementY2 = elementY1 + element.offsetHeight;

	if (x > elementX1 && x < elementX2 && y > elementY1 && y < elementY2) {
		return true;
	} else {
		return false;
	}
}

function RGB2HTML(red, green, blue) {
	var decColor = 0x1000000 + blue + 0x100 * green + 0x10000 * red;
	return '#' + decColor.toString(16).substr(1);
}

function color(R, G, B, A) {
	this.R = R;
	this.G = G;
	this.B = B;
	this.A = A;
}

function getWidth() {
	return document.documentElement.clientWidth;
}

function getHeight() {
	return document.documentElement.clientHeight;
}

function handleColorClick(selectedColor) {
	pixelColor = selectedColor;
	closeMenu();
}
