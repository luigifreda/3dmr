# -*- python -*-
# Copyright (C) 2008-2018 LAAS-CNRS, JRL AIST-CNRS.
# Author: Joseph Mirabel
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from omniidl_be.python import *
from omniidl_be.python import run as run_parent
from omniidl import idlvisitor, idlast, idltype

def _rreplace(s, old, new, occurrence):
    li = s.rsplit(old, occurrence)
    return new.join(li)

class CommentToConstVisitor (idlvisitor.AstVisitor):
    def __init__(self):
        import re
        self.commentStart  = re.compile (r"^//+ ?")

    def _commentToConst (self, node, comments):
        import re
        texts =  []
        line = node.line()
        # TODO here, we only use the comments before.
        comment = node
        for c in reversed(comments):
            if line - c.line() != 1:
                break
            line = c.line()
            comment = c
            text = c.text()
            texts.append (re.sub(self.commentStart, "", text))
        if len(texts)==0:
            if isinstance(node, idlast.Attribute):
                for i in node.identifiers():
                    print (i + " documentation may be ill-formed.")
            else:
                print (node.identifier() + " documentation may be ill-formed.")
        texts.reverse()
        # Extract the prototype of the function
        try:
            if isinstance(node, idlast.Operation):
                with open(node.file()) as fp:
                    for _ in range(node.line()-1): fp.readline()
                    line = fp.readline()
                    texts.append("\nPrototype:\t"+ line)
                    while line != "":
                        if ';' in line:
                            break
                        line = fp.readline()
                        texts.append("           \t"+ line)
        except:
            pass
        if len(texts)==0: return None
        text = ''.join (texts)
        id = node.identifier() + "__doc__"
        return idlast.Const (
                comment.file(),
                comment.line(),
                node.mainFile(),
                node.pragmas(),
                [], # comments
                id, #identifier
                node.scopedName()[:-1] + [ id, ], #scopedName
                _rreplace (node.repoId(), node.identifier(), id, 1), # repoId
                idltype.String (0), # constType, 0 means unbounded
                idltype.tk_string, # constKind
                text # value
                )
    def _addDoc (self, parent, node):
        #Commented in order to add the prototype to the documentation.
        #if len(node.comments()) > 0:
        if True:
            const = self._commentToConst (node, node.comments())
            if const is None: return
            if isinstance(parent, idlast.Module):
                parent._Module__definitions.append(const)
            elif isinstance(parent, idlast.Interface):
                parent._Interface__declarations.append(const)
                parent._Interface__contents.append(const)
            elif isinstance(parent, idlast.AST):
                parent._AST__declarations.append(const)
            else:
                print ("Doc ignored: " + comment.text())


    def visitAST(self, node):
        for n in node.declarations():
            if not output_inline and not n.mainFile(): continue

            if isinstance(n, idlast.Module) or isinstance(n, idlast.Interface):
                self._addDoc (node, n)
                n.accept(self)

    def visitModule(self, node):
        for n in node.definitions():
            if not output_inline and not n.mainFile(): continue

            if isinstance(n, idlast.Module) or isinstance(n, idlast.Interface):
                self._addDoc (node, n)
                n.accept(self)

    def visitInterface(self, node):
        for c in node.callables():
            self._addDoc (node, c)

def run(tree, args):
    ccv = CommentToConstVisitor ()
    tree.accept(ccv)

    run_parent (tree, args)
