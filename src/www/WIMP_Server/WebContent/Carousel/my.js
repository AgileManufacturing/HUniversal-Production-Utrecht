/*
 * Global variables
 */
var transformProp = Modernizr.prefixed('transform');
var carousel = 'undefined';
var SWIPE_MIN_DISTANCE = 'undefined';
var lastEvent;
var panelPlaceLeftRect = 'undefined';
var panelPlaceLeftWidth = 'undefined';
var panelPlaceLeftHeight = 'undefined';
var panelPlaceRightRect = 'undefined';
var panelPlaceRightWidth = 'undefined';
var panelPlaceRightHeight = 'undefined';

/*
 * Carousel functions
 */

function createNewCarousel(el) {
	carousel = new Object();
	carousel.element = el;
	carousel.rotation = 0;
	carousel.panelCount = 0;
	carousel.theta = 0;
	carousel.isHorizontal = true;
}

function modifyCarousel() {
	var panel, angle, i;

	carousel.panelSize = carousel.element[carousel.isHorizontal ? 'offsetWidth' : 'offsetHeight'];
	carousel.rotateFn = carousel.isHorizontal ? 'rotateY' : 'rotateX';
	carousel.theta = 360 / carousel.panelCount;

	// do some trig to figure out how big the carousel
	// is in 3D space
	carousel.radius = Math.round((carousel.panelSize / 2) / Math.tan(Math.PI / carousel.panelCount));

	for ( i = 0; i < carousel.panelCount; i++) {
		panel = carousel.element.children[i];
		angle = carousel.theta * i;
		panel.style.opacity = 1;
		// panel.style.backgroundColor = 'hsla(' + angle + ', 100%, 50%, 0.8)';
		// rotate panel, then push it out in 3D space
		panel.style[transformProp] = carousel.rotateFn + '(' + angle + 'deg) translateZ(' + carousel.radius + 'px)';
	}

	// hide other panels
	for (; i < carousel.totalPanelCount; i++) {
		panel = carousel.element.children[i];
		panel.style.opacity = 0;
		panel.style[transformProp] = 'none';
	}

	// adjust rotation so panels are always flat
	carousel.rotation = Math.round(carousel.rotation / carousel.theta) * carousel.theta;

	transformCarousel();
};

function transformCarousel() {
	// push the carousel back in 3D space,
	// and rotate it
	carousel.element.style[transformProp] = 'translateZ(-' + carousel.radius + 'px) ' + carousel.rotateFn + '(' + carousel.rotation + 'deg)';
};

function getCarouselPanelCount() {
	var car = document.getElementById('carousel');
	var figures = car.getElementsByClassName("figure");
	return figures.length;
}

/*
 * Init functions
 */

function init() {
	initContainerSize();
	initFigureSize();
	initTopOffset();

	window.parent.hideCreateTab();

	if (carousel == 'undefined') {
		createNewCarousel(document.getElementById('carousel'));
	}

	// populate on startup
	carousel.panelCount = getCarouselPanelCount();
	modifyCarousel();

	setTimeout(function() {
		document.body.addClassName('ready');
	}, 0);
};

function initFigureSize() {
	var figures = document.getElementsByClassName('figure');
	var car = document.getElementById('carousel');
	for (var i = 0; i < figures.length; i++) {
		figures[i].style.width = car.offsetWidth - 20 + 'px';
		figures[i].style.height = car.offsetHeight - 20 + 'px';
	}
}

function initContainerSize() {
	var con = document.getElementById('container');
	var size = Math.min(window.innerWidth, window.innerHeight);
	size = (size / 100) * 90;
	con.style.width = size + 'px';
	con.style.height = size + 'px';
	SWIPE_MIN_DISTANCE = size / 4;
}

function initTopOffset() {
	var topoffset = document.getElementById('topoffset');
	var con = document.getElementById('container');
	topoffset.style.height = ((window.innerHeight - con.offsetHeight) / 2) + 'px';
}

/*
 * Event functions
 */

function DownEventHandler(event) {
	lastEvent = event;
}

function UpEventHandler(event) {
	if (isForwardFling(lastEvent.clientX, lastEvent.clientY, event.clientX, event.clientY)) {
		carousel.rotation += carousel.theta * 1 * -1;
		transformCarousel();
	} else if (isBackwardFling(lastEvent.clientX, lastEvent.clientY, event.clientX, event.clientY)) {
		carousel.rotation += carousel.theta * -1 * -1;
		transformCarousel();
	} else if (isAClick(lastEvent.clientX, lastEvent.clientY, event.clientX, event.clientY)) {
		if (isClickOnLeftArrow(event.clientX, event.clientY)) {
			carousel.rotation += carousel.theta * 1 * -1;
			transformCarousel();
		} else if (isClickOnRightArrow(event.clientX, event.clientY)) {
			carousel.rotation += carousel.theta * -1 * -1;
			transformCarousel();
		} else {
			launchApplication(getShowingPanelNumber());
		}
	}
}

function MoveEventHandler(event) {

}

function isForwardFling(startX, startY, endX, endY) {
	if (Math.abs(startY - endY) < SWIPE_MIN_DISTANCE) {
		var between = startX - endX;
		if (between > SWIPE_MIN_DISTANCE) {
			return true;
		} else {
			return false;
		}
	} else {
		return false;
	}
}

function isBackwardFling(startX, startY, endX, endY) {
	if (Math.abs(startY - endY) < SWIPE_MIN_DISTANCE) {
		var between = startX - endX;
		if (between < -SWIPE_MIN_DISTANCE) {
			return true;
		} else {
			return false;
		}
	} else {
		return false;
	}
}

function isAClick(startX, startY, endX, endY) {
	if (startX == endX && startY == endY) {
		return true;
	} else {
		return false;
	}
}

function isClickOnLeftArrow(x, y) {
	if (panelPlaceLeftRect == 'undefined') {
		var lefts = document.getElementsByClassName('toLeft');
		panelPlaceLeftRect = lefts[0].getBoundingClientRect();
		panelPlaceLeftWidth = lefts[0].width;
		panelPlaceLeftHeight = lefts[0].height;
	}

	if (x > panelPlaceLeftRect.left && x < panelPlaceLeftRect.left + panelPlaceLeftWidth && y > panelPlaceLeftRect.top && y < panelPlaceLeftRect.top + panelPlaceLeftHeight) {
		return true;
	} else {
		return false;
	}
}

function isClickOnRightArrow(x, y) {
	if (panelPlaceRightRect == 'undefined') {
		var rights = document.getElementsByClassName('toRight');
		panelPlaceRightRect = rights[0].getBoundingClientRect();
		panelPlaceRightWidth = rights[0].width;
		panelPlaceRightHeight = rights[0].height;
	}

	if (x > panelPlaceRightRect.left && x < panelPlaceRightRect.left + panelPlaceRightWidth && y > panelPlaceRightRect.top && y < panelPlaceRightRect.top + panelPlaceRightHeight) {
		return true;
	} else {
		return false;
	}
}

/*
 * Other
 */

function detectmob() {
	if (navigator.userAgent.match(/Android/i) || navigator.userAgent.match(/webOS/i) || navigator.userAgent.match(/iPhone/i) || navigator.userAgent.match(/iPad/i) || navigator.userAgent.match(/iPod/i) || navigator.userAgent.match(/BlackBerry/i) || navigator.userAgent.match(/Windows Phone/i)) {
		return true;
	} else {
		return false;
	}
}

function getShowingPanelNumber() {
	var panel = carousel.rotation * -1;
	panel = panel / carousel.theta;
	panel = panel % carousel.panelCount;

	if (panel < 0) {
		panel = panel + carousel.panelCount;
	}

	return panel;
}

function launchApplication(panelNR) {
	switch(panelNR) {
		case 0:
			window.location = "../pickAndPlace/index.html";
			break;
		case 1:
			window.location = "../stacking/index.html";
			break;
		case 2:
			window.location = "../Paint/index.html";
			break;
		case 3:
			window.location = "../stlloader/index.html";
			break;
		default:
			alert("Panel " + panelNR + " not implemented yet");
			break;
	}
}