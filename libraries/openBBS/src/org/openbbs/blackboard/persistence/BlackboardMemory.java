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

import java.util.Iterator;

import org.openbbs.blackboard.EntryFilter;
import org.openbbs.blackboard.Zone;
import org.openbbs.blackboard.ZoneSelector;

/**
 * A BlackboardMemory is responsbile for storing the entries
 * on a blackboard. Similar to blackboard, entries cannot be
 * modified or replaced while stored in a memory. Instead, a
 * memory provides two essential primitive operations to store
 * and remove entries. Replacing an object would be to remove
 * it first and store the replacement entry afterwards. Similar,
 * to change an entry, it has to removed first, then modifications
 * can be done, and then the entry has to be stored again.
 */
public interface BlackboardMemory
{
   /**
    * Create a new zone in the memory. The zone must not
    * exist.
    * 
    * @param zone  a non-null Zone to create. Implementors
    *              may restrict the possible Zone implementations.
    */
   public void createZone(Zone zone);

   /**
    * Remove an existing zone from the memory. All entries
    * in this zone are also removed.
    * 
    * @param zone  a non-null, existing Zone.
    */
   public void dropZone(Zone zone);

   /**
    * Check wether a particular zone exists in the memory.
    * 
    * @param zone  a non-null Zone.
    */
   public boolean zoneExists(Zone zone);

   /**
    * Store a new entry inside a zone of the memory. No entry
    * which is equal to the specified entry must exist in any
    * zone of the memory.
    * 
    * @param zone   the zone in which the entry should be stored.
    *               Implementors may place specific constraints on
    *               zones. Not null.
    * @param entry  the entry to be stored. Implementors may place
    *               specific constraints on entries, such as that
    *               an entry must be Serializable. Not null.
    */
   public void storeEntry(Zone zone, Object entry);

   /**
    * Remove an entry from the memory. An entry which is equal to
    * the specified entry must exist and will be removed.
    * 
    * @param entry  the entry to be removed.
    */
   public void removeEntry(Object entry);

   /**
    * Check whether a given entry exists in the memory.
    * 
    * @param entry  a non-null entry.
    */
   public boolean entryExists(Object entry);

   /**
    * Get the zone in which a particular entry is stored. Returns
    * null if the entry is not stored in this memory.
    * 
    * @param entry  a non-null entry.
    */
   public Zone getZone(Object entry);

   /**
    * Get an Iterator over all entries in the memory which reside
    * in a zone that is machted by the specified {@link ZoneSelector}
    * and is selected by the specified {@EntryFilter}. If no such entry
    * is found, the returned Iterator is empty.
    * 
    * @param zoneSelector  a non-null ZoneSelector. Implementors may
    *                      restrict the possible ZoneSelectors to specific
    *                      implementations.
    * @param entryFilter   a non-null EntryFilter. Implementors may
    *                      restrict the possible EntryFilters to specific
    *                      implementations.
    */
   public Iterator<Object> getEntries(ZoneSelector zoneSelector, EntryFilter entryFilter);
}
