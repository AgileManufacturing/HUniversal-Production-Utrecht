/**
 *
 * Project: Dummy Product Agent
 *
 * Package: newDataClasses
 *
 * File: EquipletMapper.java
 *
 * Author: Mike Schaap
 *
 * Version: 1.0
 *
 */
package newDataClasses;

import java.util.ArrayList;
import java.util.HashMap;

import jade.core.AID;

public abstract class Mapper<KT,VT> {

	protected HashMap<KT, ArrayList<VT>> _items;
	
	protected Mapper() {
		this._items = new HashMap<KT, ArrayList<VT>>();
	}
	
	protected void addKey(KT id) {
		this._items.put(id, new ArrayList<VT>());
	}
	
	protected void removeKey(KT id) {
		this._items.remove(id);
	}
	
	protected ArrayList<VT> getValues(KT id) {
		return this._items.get(id);
	}
	
	protected void addValue(KT id, VT value) {
		ArrayList<VT> tmp = this._items.get(id);
		tmp.add(value);
		this._items.put(id, tmp);
	}
	
	protected void removeValue(KT id, VT value) {
		ArrayList<VT> tmp = this._items.get(id);
		tmp.remove(value);
		this._items.put(id, tmp);
	}
	
	
	
	
}
