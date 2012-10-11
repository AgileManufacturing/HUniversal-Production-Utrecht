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
package org.openbbs.blackboard.persistence;

import java.io.Serializable;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;

import org.apache.commons.lang.Validate;
import org.openbbs.blackboard.EntryFilter;
import org.openbbs.blackboard.Zone;
import org.openbbs.blackboard.ZoneSelector;
import org.openbbs.blackboard.persistence.snapshot.SnapshottableMemory;

/**
 * A default BlackboardMemory implementation that stores all entry
 * in main memory. Hence, the number of entries which can be stored
 * is limited by the size of the main memory. Because main memory is
 * volatile, the state of the memory is lost when the system terminates.
 * If you need persistence, you have to take care of this by yourself.
 * A very simple solution would be to take a snapshot of the memory at
 * certain points. This is supported through implementation of the
 * {@link SnapshottableMemory} interface. Thereby it is easy to create
 * snapshots by using a {@link Snapshotter}.
 */
public class TransientMemory implements BlackboardMemory, SnapshottableMemory, Serializable
{
   private Map<Zone, Set<Object>> zones = new HashMap<Zone, Set<Object>>();
   private Map<Object, Zone> entries = new HashMap<Object, Zone>();

   /**
    * Remove all zones and entries from the memory. If this method fails,
    * the state of the memory is undefined.
    */
   public void clear() {
      this.zones.clear();
      this.entries.clear();
   }

   /**
    * @see BlackboardMemory#createZone(Zone)
    */
   public void createZone(Zone zone) {
      Validate.notNull(zone, "zone must not be null");

      if (zones.containsKey(zone)) {
         throw new BlackboardMemoryException("zone " + zone + " already exists");
      }
      this.zones.put(zone, new HashSet<Object>());
   }

   /**
    * @see BlackboardMemory#dropZone(Zone)
    */
   public void dropZone(Zone zone) {
      Validate.notNull(zone, "zone must not be null");

      if (!zones.containsKey(zone)) {
         throw new BlackboardMemoryException("zone " + zone + " does not exist");
      }

      for (Object entry : this.zones.remove(zone)) {
         this.entries.remove(entry);
      }
   }

   /**
    * @see BlackboardMemory#zoneExists(Zone)
    */
   public boolean zoneExists(Zone zone) {
      return this.zones.containsKey(zone);
   }

   /**
    * @see BlackboardMemory#getEntries(ZoneSelector, EntryFilter)
    */
   public Iterator<Object> getEntries(ZoneSelector zoneSelector, EntryFilter entryFilter) {
      return new EntryIterator(zoneSelector, entryFilter);
   }

   /**
    * @see BlackboardMemory#storeEntry(Zone, Object)
    */
   public void storeEntry(Zone zone, Object entry) {
      Validate.notNull(entry, "cannot store null entry");
      Validate.notNull(zone, "cannot store entry in null-zone");

      if (this.entries.containsKey(entry)) {
         throw new BlackboardMemoryException("cannot store entry " + entry + " twice inside the same memory");
      }

      Set<Object> objectsInZone = this.zones.get(zone);
      Validate.notNull(objectsInZone, "zone " + zone + " is unknown");
      objectsInZone.add(entry);
      entries.put(entry, zone);
   }

   /**
    * @see BlackboardMemory#entryExists(Object)
    */
   public boolean entryExists(Object entry) {
      Validate.notNull(entry, "cannot check existence of a null-entry");
      return this.entries.containsKey(entry);
   }

   /**
    * @see BlackboardMemory#getZone(Object)
    */
   public Zone getZone(Object entry) {
      Validate.notNull(entry, "cannot get the zone of a null-entry");
      return this.entries.get(entry);
   }

   /**
    * @see BlackboardMemory#removeEntry(Object)
    */
   public void removeEntry(Object entry) {
      Validate.notNull(entry, "cannot remove null-entry from memory");
      Zone zone = this.entries.remove(entry);
      Validate.notNull(zone, "removed entry has no zone");
      Validate.isTrue(this.zones.get(zone).remove(entry), "failed to remove entry from zone");
   }

   /**
    * @see SnapshottableMemory#getEntriesInZone(Zone)
    */
   public Iterator<Object> getEntriesInZone(Zone zone) {
      Validate.notNull(zone);

      if (!this.zones.containsKey(zone)) {
         throw new BlackboardMemoryException("unknown zone " + zone);
      }

      return Collections.unmodifiableCollection(this.zones.get(zone)).iterator();
   }

   /**
    * @see SnapshottableMemory#getZonesIterator()
    */
   public Iterator<Zone> getZonesIterator() {
      return Collections.unmodifiableCollection(this.zones.keySet()).iterator();
   }

   /**
    * @see SnapshottableMemory#restoreEntry(Object, Zone)
    */
   public void restoreEntry(Object entry, Zone zone) {
      this.storeEntry(zone, entry);
   }

   /**
    * @see SnapshottableMemory#restoreZone(Zone)
    */
   public void restoreZone(Zone zone) {
      this.createZone(zone);
   }

   /**
    * An Iterator over all entries which are matched by an EntryFilter
    * and which reside a zone selected by a ZoneSelector.
    */
   private class EntryIterator implements Iterator<Object>
   {
      private Iterator<Zone> zoneIterator = null;
      private Iterator<Object> entryIterator = null;
      private final EntryFilter entryFilter;
      private Object nextEntry = null;

      public EntryIterator(ZoneSelector zoneSelector, EntryFilter entryFilter) {
         Validate.notNull(entryFilter);
         this.zoneIterator = new ZoneIterator(zoneSelector);
         this.entryFilter = entryFilter;
      }

      public boolean hasNext() {
         if (this.nextEntry != null) {
            return true;
         }
         return this.updateNextEntry();
      }

      public Object next() {
         if (this.nextEntry == null) {
            if (!this.updateNextEntry()) {
               throw new IllegalStateException("no more entries to iterate over");
            }
         }

         Validate.notNull(this.nextEntry, "nextEntry is not set");
         Object entry = this.nextEntry;
         this.nextEntry = null;
         return entry;
      }

      public void remove() {
         throw new UnsupportedOperationException("remove is not supported by this iterator");
      }

      private boolean updateNextEntry() {
         this.nextEntry = null;

         if (this.entryIterator == null) {
            this.gotoNextZone();
         }

         while (this.entryIterator != null && this.nextEntry == null) {
            while (this.entryIterator.hasNext()) {
               Object candidate = this.entryIterator.next();
               if (this.entryFilter.selects(candidate)) {
                  this.nextEntry = candidate;
                  break;
               }
            }

            if (this.nextEntry == null) {
               this.gotoNextZone();
            }
         }

         return this.nextEntry != null;
      }

      private void gotoNextZone() {
         if (!this.zoneIterator.hasNext()) {
            this.entryIterator = null;
            return;
         }

         Zone nextZone = this.zoneIterator.next();
         this.entryIterator = zones.get(nextZone).iterator();
      }
   }

   /**
    * An Iterator over all Zones which are selected by a ZoneSelector.
    */
   private class ZoneIterator implements Iterator<Zone>
   {
      private final ZoneSelector zoneSelector;
      private final Iterator<Zone> allZonesIterator;
      private Zone nextZone = null;

      public ZoneIterator(ZoneSelector zoneSelector) {
         Validate.notNull(zoneSelector);
         this.zoneSelector = zoneSelector;
         this.allZonesIterator = zones.keySet().iterator();
      }

      public boolean hasNext() {
         if (this.nextZone != null) {
            return true;
         }
         return this.updateNextZone();
      }

      public Zone next() {
         if (this.nextZone == null) {
            if (!this.updateNextZone()) {
               throw new IllegalStateException("no more zones to iterate over");
            }
         }

         Validate.notNull(this.nextZone, "nextZone is not set");
         Zone zone = this.nextZone;
         this.nextZone = null;
         return zone;
      }

      public void remove() {
         throw new UnsupportedOperationException("remove is not supported by this iterator");
      }

      private boolean updateNextZone() {
         this.nextZone = null;
         while (this.allZonesIterator.hasNext()) {
            Zone candiate = this.allZonesIterator.next();
            if (this.zoneSelector.selects(candiate)) {
               this.nextZone = candiate;
               break;
            }
         }
         return this.nextZone != null;
      }
   }

   private static final long serialVersionUID = -7853510868367120188L;
}
