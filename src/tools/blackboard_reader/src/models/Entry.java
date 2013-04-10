/**
 * @author Ammar Abdulamir
 * @file Entry.java
 * @brief A blackboard entry.
 * @date Created: 4/10/13
 * @section LICENSE
 * License: newBSD
 * Copyright © 2013, HU University of Applied Sciences Utrecht.
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

package models;

import com.mongodb.BasicDBObject;

import javax.swing.event.TreeModelListener;
import javax.swing.tree.TreeModel;
import javax.swing.tree.TreePath;
import java.util.Map;

/**
 * A blackboard entry.
 **/
public class Entry implements TreeModel {
    //protected File root;
    //private ArrayList<BasicDBObject> entries = new ArrayList<BasicDBObject>();
    private BasicDBObject root;

    public Entry(BasicDBObject entry) {
        //root = new File("C:/");
        root = entry;
    }

    public void add(BasicDBObject entry) {
        //  entries.add(entry);
    }

    @Override
    public Object getRoot() {
        return root;
    }

    @Override
    public Object getChild(Object parent, int index) {
        Object child = null;

        if (parent instanceof BasicDBObject) {
            BasicDBObject object = (BasicDBObject) parent;
            if (index >= object.size()) return null;

            int i = 0;
            for (Map.Entry<String, Object> entry : object.entrySet()) {
                child = entry.getValue();
                if (i++ == index)
                    break;
            }
        }

        return child;
    }

    @Override
    public int getChildCount(Object parent) {
        int children = 0;
        if (parent instanceof BasicDBObject)
            children = ((BasicDBObject) parent).size();

        return children;
    }

    @Override
    public boolean isLeaf(Object node) {
        return !(node instanceof BasicDBObject);
    }

    @Override
    public void valueForPathChanged(TreePath path, Object newValue) {
    }

    @Override
    public int getIndexOfChild(Object parent, Object child) {
        if (parent instanceof BasicDBObject) {
            BasicDBObject object = (BasicDBObject) parent;
            for (int i = 0; i < object.size(); i++) {
                if (object.equals(child)) {
                    System.out.println("getIndexOfChild " + child);
                    return i;
                }
            }
        }

        System.out.println("getIndexOfChild -1");
        return -1;
    }

    @Override
    public void addTreeModelListener(TreeModelListener l) {
    }

    @Override
    public void removeTreeModelListener(TreeModelListener l) {
    }
}
