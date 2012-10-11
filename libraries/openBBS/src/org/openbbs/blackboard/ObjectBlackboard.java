/*
 *  Copyright [2006-2007] [Stefan Kleine Stegemann]
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
package org.openbbs.blackboard;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;

import org.apache.commons.lang.Validate;
import org.openbbs.blackboard.persistence.BlackboardMemory;
import org.openbbs.blackboard.persistence.TransientMemory;

/**
 * An ObjectBlackboard holds arbritrary objects.
 *
 * @author sks
 */
public class ObjectBlackboard implements Blackboard
{
   private final Map<BlackboardObserver, ZoneSelector> observers = new HashMap<BlackboardObserver, ZoneSelector>();
   private CloneStrategy cloneStrategy;
   private BlackboardMemory memory;

   public ObjectBlackboard() {
      this(new CloneByMethodStrategy("clone"));
   }

   public ObjectBlackboard(CloneStrategy cloneStrategy) {
      this.setCloneStrategy(cloneStrategy);
      this.setMemory(new TransientMemory());
      this.openZone(Zone.DEFAULT);
   }

   public void setCloneStrategy(CloneStrategy cloneStrategy) {
      Validate.notNull(cloneStrategy);
      this.cloneStrategy = cloneStrategy;
   }

   public void setMemory(BlackboardMemory memory) {
      Validate.notNull(memory);
      this.memory = memory;
   }

   public synchronized void openZone(Zone zone) {
      Validate.notNull(zone);

      if (this.memory.zoneExists(zone)) {
         throw new BlackboardZoneException("zone " + zone.name() + " is already open");
      }
      this.memory.createZone(zone);
   }

   public synchronized void closeZone(Zone zone) {
      Validate.notNull(zone);

      if (!this.memory.zoneExists(zone)) {
         throw new BlackboardZoneException("zone " + zone.name() + " is unknown");
      }
      this.memory.dropZone(zone);
   }

   public synchronized void write(Zone zone, Object entry) {
      Validate.notNull(entry);

      if (!this.knowsZone(zone)) throw new BlackboardZoneException("zone " + zone.name() + " is unknown");

      if (this.memory.entryExists(entry)) {
         throw new WriteBlackboardException("entry " + entry.toString() + " is already present on this blackboard");
      }

      Object clonedEntry = this.cloneStrategy.clone(entry);
      this.memory.storeEntry(zone, clonedEntry);

      this.notifyEntryAdded(zone, clonedEntry);
   }

   public Zone zoneOf(Object entry) {
      Validate.notNull(entry, "cannot determine zone of null entry");

      Zone zone = this.memory.getZone(entry);
      if (zone == null) {
         throw new ReadBlackboardException("unknown entry " + entry.toString());
      }

      return zone;
   }

   public synchronized Object read(ZoneSelector zoneSelector, EntryFilter filter) {
      Validate.notNull(zoneSelector);
      Validate.notNull(filter);

      Iterator<Object> it = this.memory.getEntries(zoneSelector, filter);
      if (!it.hasNext()) {
         return null;
      }

      return this.cloneStrategy.clone(it.next());
   }
      

   public synchronized Set<Object> readAll(ZoneSelector zoneSelector, EntryFilter filter) {
      Validate.notNull(zoneSelector);
      Validate.notNull(filter);

      Set<Object> entries = new HashSet<Object>();
      for (Iterator<Object> it = this.memory.getEntries(zoneSelector, filter); it.hasNext();) {
         entries.add(this.cloneStrategy.clone(it.next()));
      }

      return entries;
   }

   public boolean exists(ZoneSelector zoneSelector, EntryFilter filter) {
      return this.read(zoneSelector, filter) != null;
   }

   public synchronized Object take(ZoneSelector zoneSelector, EntryFilter filter) {
      Validate.notNull(zoneSelector);
      Validate.notNull(filter);

      Iterator<Object> it = this.memory.getEntries(zoneSelector, filter);
      if (!it.hasNext()) {
         throw new ReadBlackboardException("no entry selected");
      }

      Object takenEntry = it.next();
      Zone zoneOfTakenEntry = this.memory.getZone(takenEntry);
      this.memory.removeEntry(takenEntry);

      this.notifyEntryRemoved(zoneOfTakenEntry, takenEntry);

      return takenEntry;
   }

   public void registerInterest(ZoneSelector zoneSelector, BlackboardObserver observer) {
      Validate.notNull(zoneSelector);
      Validate.notNull(observer);
      this.observers.put(observer, zoneSelector);
   }

   private boolean knowsZone(Zone zone) {
      Validate.notNull(zone);
      return this.memory.zoneExists(zone);
   }
   
   public boolean isZoneOpen(Zone zone)
   {
	   Validate.notNull(zone);
	   return this.memory.zoneExists(zone);	   
   }

   private void notifyEntryAdded(Zone zone, Object entry) {
      for (BlackboardObserver observer : this.observers.keySet()) {
         if (this.isObserverInterstedInZone(observer, zone)) observer.blackboardDidAddEntry(this, zone, entry);
      }
   }

   private void notifyEntryRemoved(Zone zone, Object entry) {
      for (BlackboardObserver observer : this.observers.keySet()) {
         if (this.isObserverInterstedInZone(observer, zone)) observer.blackboardDidRemoveEntry(this, zone, entry);
      }
   }

   private boolean isObserverInterstedInZone(BlackboardObserver observer, Zone zone) {
      Validate.notNull(observer);
      Validate.notNull(zone);

      ZoneSelector zoneSelectorForObserver = this.observers.get(observer);
      Validate.notNull(zoneSelectorForObserver, "observer " + observer + " has no ZoneSelector");
      return zoneSelectorForObserver.selects(zone);
   }

}
