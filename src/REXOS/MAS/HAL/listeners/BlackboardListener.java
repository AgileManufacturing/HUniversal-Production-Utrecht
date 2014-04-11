package HAL.listeners;



public interface BlackboardListener {
	
	public void OnEquipleStateChanged(String equipletName, String state);
	public void OnEquipleModeChanged(String equipletName, String mode);
	public void onProcessStatusChanged(String status);
	public void onModuleStateChanged(String state);
	public void onModuleModeChanged(String mode);

}
