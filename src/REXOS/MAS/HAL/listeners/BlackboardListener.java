package HAL.listeners;



public interface BlackboardListener {
	
	public void OnEquipleStateChanged(String id, String state);
	public void OnEquipleModeChanged(String id, String mode);
	public void onProcessStateChanged(String state);
	public void onModuleStateChanged(String state);
	public void onModuleModeChanged(String mode);

}
