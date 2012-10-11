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
package org.openbbs.blackboard.persistence.snapshot;

import java.util.Iterator;

import org.openbbs.blackboard.Zone;

/**
 * The contents of a SnapshottableMemory can be saved to a
 * snasphot file and restored from such a file. Use a Snapshotter
 * to create such a snapshot file.
 */
public interface SnapshottableMemory
{
   /**
    * Returns a non-null Iterator over all zones in the memory (including
    * the emtpy ones.
    */
   public Iterator<Zone> getZonesIterator();

   /**
    * Returns an iterator over all entries stored in a particular
    * zone inside the memory. If the zone is empty, it is both ok,
    * to return null or an Iterator which returns no object.
    * 
    * @param zone  a non-null Zone.
    */
   public Iterator<Object> getEntriesInZone(Zone zone);

   /**
    * Restore a Zone in the memory. This method is called when the
    * contents of a memory are restored from a snapshot file. The
    * entries will follow next, if any.
    * 
    * @param zone  the non-null Zone to be restored.
    */
   public void restoreZone(Zone zone);

   /**
    * Restore an entry in a Zone. It is guaranteed that the zone has
    * been restored before.
    * 
    * @param entry  a non-null entry to be restored.
    * @param zone   the non-null zone in which the restored entry resides.
    */
   public void restoreEntry(Object entry, Zone zone);
}
