<?php
require_once(__DIR__.'/../../lang/Languages.class.php');

define ('INPUT_NUMBER', 'INPUT_NUMBER');
define ('INPUT_SELECT', 'INPUT_SELECT');

class Service {
	private $inputs;
	private $service;
	
	function __construct($service) {
    	$this->service = $service;
		$this->inputs = array();
   	}
	
	public function addInput($text, $input, $attributes, $before, $after, $options=array()){
		$this->inputs[$input][] = array('text' => $text, 'attributes' => $attributes, 'before' => $before, 'after' => $after, 'options' => $options);
	}
	
	public function buildService(){
		$html = '<form id="draw_container" class="form-horizontal" role="form">';
		$html .= '<div class="col-sm-6">';
	
		foreach ($this->inputs as $type => $type_input){
			foreach ($type_input as $input){
				$html .= '<div class="form-group">';
				$html .= '<label for="service_selector" class="col-sm-6 control-label" style="text-align:left">';
				$html .= $input['text'];
				$html .= '</label>';
				$html .= '<div class="col-sm-6">';
				if ($type != INPUT_SELECT){
					$html .= '<div class="input-group">';
				}
				if ($input['before'] != ''){
					$html .= '<span class="input-group-addon">' . $input['before'] . '</span>';
				}
				
				//Input types
				if ($type == INPUT_NUMBER){
					$html .= '<input type="number" class="form-control" ';
					foreach ($input['attributes'] as $key => $attribute){
						$html .= $key . '="' . $attribute . '" ';
					}
					$html .= '/>';
				}
				else if ($type == INPUT_SELECT){
					
					$html .= '<select class="form-control" ';
					foreach ($input['attributes'] as $key => $attribute){
						$html .= $key . '="' . $attribute . '" ';
					}
					$html .= '>';
					
					foreach ($input['options'] as $option){
						$html .= '<option>' . $option . '</option>';
					}
					
					$html .= '</select>';
				}
				
				if ($input['after'] != ''){
					$html .= '<span class="input-group-addon">' . $input['after'] . '</span>';
				}
				
				if ($type != INPUT_SELECT){
					$html .= '</div>';
				}
				$html .= '</div>';
				$html .= '</div>';
			}
		}
		$html .= $this->getPartSelectorByService('target', $this->service);
		$html .= $this->getPartSelectorByService('subject', $this->service);
        $html .= '</div>';
        $html .= '</form>';
		return $html;
	}
	
	public function getPartSelectorByService($type, $service){
		$num = 0;
		$html = '<div class="form-group">';
		$html .= '<label for="service_selector" class="col-sm-6 control-label" style="text-align:left">';
		if ($type == 'target'){
			$html .= Language::getText('SELECT_TARGET');
		}
		else {
			$html .= Language::getText('SELECT_SUBJECT');
		}
		$html .= '</label>';
		$html .= '<div class="col-sm-6">';
		$html .= '<select id="'.$type.'_list" onchange="loadPart(this, \''.$type.'_list_extended\')" class="form-control">';
		foreach (new DirectoryIterator(__DIR__.'/../../parts') as $fileInfo) {
			if($fileInfo->isDot() || !$fileInfo->isDir()) continue;
			
			$path = $fileInfo->getPathname().'/'.$fileInfo->getFilename();
			$classpath = $path.'.class.php';
			
			$classname = ucwords(strtolower(pathinfo($path.'.php', PATHINFO_FILENAME)));
			if ($classname == 'Part') continue;
			require_once($classpath);
			$part = new $classname();
			
			if ($type == 'target'){
				if (!$part->isServiceTarget($service)){
					continue;
				}
			}
			else {
				if (!$part->isServiceSubject($service)){
					continue;
				}
			}
			$num++;
			$html .= '<option>' . $part->getName() . '</option>';
		}
		$html .= '</select>';
		$html .= '</div>';
		$html .= '</div>';
		$html .= '<div id="'.$type.'_list_extended"></div>';
		$html .= '<script>loadPart(document.getElementById("'.$type.'_list"), \''.$type.'_list_extended\');</script>';
		if ($num == 0) return '';
		return $html;
	}
}
?>