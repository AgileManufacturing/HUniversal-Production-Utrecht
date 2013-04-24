/**
 * @author Ammar Abdulamir
 * @file Entries.java
 * @brief A model for entries.
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

package rexos.client.blackboardreader.models;

import com.mongodb.BasicDBObject;

import javax.swing.event.TreeModelListener;
import javax.swing.tree.TreeModel;
import javax.swing.tree.TreePath;
import java.util.ArrayList;
import java.util.Map;

/**
 * A model for entries.
 **/
public class Entries implements TreeModel {
    /**
     * A list of entries.
     *
     * @var ArrayList root
     **/
    private final ArrayList<EntryNode> root;

    /**
     * The constructor.
     **/
    public Entries() {
        root = new ArrayList<EntryNode>();
    }

    /**
     * Adds an entry to the list.
     *
     * @param node An entry.
     */
    public void add(EntryNode node) {
        root.add(node);
    }

    /**
     * Gets the root node.
     *
     * @return The root node.
     */
    @Override
    public Object getRoot() {
        return root;
    }

    /**
     * Gets the child of parent at index in the parent's child array.
     *
     * @param parent The parent.
     * @param index The index.
     * @return Returns the child of a parent at a specific index.
     **/
    @Override
    public Object getChild(Object parent, int index) {
        if (parent instanceof ArrayList)
            return root.get(index);
        else {
            BasicDBObject bdo = ((EntryNode) parent).getNodeAsBdo();
            if (index >= bdo.size()) return null;

            int i = 0;
            for (Map.Entry<String, Object> entry : bdo.entrySet()) {
                if (i++ == index)
                    return new EntryNode(entry.getKey(), entry.getValue());
            }

            return null;
        }
    }

    /**
     * Returns the number of children of parent.
     *
     * @param parent The parent.
     * @return The number of children the parent has.
     **/
    @Override
    public int getChildCount(Object parent) {
        int children;

        if (parent instanceof ArrayList)
            children = root.size();
        else if (parent instanceof EntryNode && ((EntryNode) parent).isBdoNode())
            children = ((EntryNode) parent).getNodeAsBdo().size();
        else
            children = 0;

        return children;
    }

    /**
     * Check if node is a leaf.
     *
     * @param node The node
     * @return Returns whether the node is a leaf or not.
     **/
    @Override
    public boolean isLeaf(Object node) {
        return !(node instanceof ArrayList || node instanceof EntryNode && ((EntryNode) node).isBdoNode() && ((EntryNode) node).getNodeAsBdo().size() > 0);
    }

    @Override
    public void valueForPathChanged(TreePath path, Object newValue) {
    }

    /**
     * Returns the index of child in parent. If either parent or child is null, returns -1.
     *
     * @param parent The parent.
     * @param child The child.
     * @return The index of the child in parent.
     */
    @Override
    public int getIndexOfChild(Object parent, Object child) {
        int index = -1;

        if (parent instanceof ArrayList)
            index = 0;
        else if (parent instanceof EntryNode && ((EntryNode) parent).isBdoNode()) {
            BasicDBObject bdo = ((EntryNode) parent).getNodeAsBdo();
            int i = 0;
            for (String key : bdo.keySet()) {
                if (key.equals((((EntryNode) child)).getKey())) break;
                i++;
            }
            index = i;
        }

        return index;
    }

    @Override
    public void addTreeModelListener(TreeModelListener l) {
    }

    @Override
    public void removeTreeModelListener(TreeModelListener l) {
    }
}