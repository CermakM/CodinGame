## A Trie structure for Telephone Numbers task

class TrieNode:
    total_nodes = -1;
    
    def __init__(self):
        TrieNode.total_nodes += 1;
        self.children = dict()

    def insert(self, num):
        node = self;
        for n in num:
            if not n in node.children:
                node.children[n] = TrieNode();
            node = node.children[n]

''' GAME LOOP ''' 
root = TrieNode(); # Root node with value of total_nodes = 0
for i in range(int(input())):
    root.insert(input());

# The number of elements (referencing a number) stored in the structure.
print(TrieNode.total_nodes);

