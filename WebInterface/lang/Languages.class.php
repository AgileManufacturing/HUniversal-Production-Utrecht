<?php

class Language {
	
	public static function getLanguage(){
		return 'GB';
	}
	public static function getText($text_id, $language = 'GB'){
		include $language . '.php';
		return $text[$text_id];
	}
}

?>