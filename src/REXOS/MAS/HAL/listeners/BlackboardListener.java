package HAL.listeners;


/**
 * 
 * @author Aristides Ayala Mendoza
 *
 */
public interface BlackboardListener {
	
	public void OnEquipletStateChanged(String equipletName, String state);
	public void OnEquipletModeChanged(String equipletName, String mode);
	public void onProcessStatusChanged(String status);
	public void onModuleStateChanged(String state);
	public void onModuleModeChanged(String mode);
	public void OnEquipletIpChanged(String ip);

}
