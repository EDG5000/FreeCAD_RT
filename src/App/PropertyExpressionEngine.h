/***************************************************************************
 *   Copyright (c) Eivind Kvedalen (eivind@kvedalen.name) 2015             *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 *                                                                         *
 *   This library  is distributed in the hope that it will be useful,      *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Library General Public License for more details.                  *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this library; see the file COPYING.LIB. If not,    *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************/

#ifndef EXPRESSIONENGINE_H
#define EXPRESSIONENGINE_H

#include <boost/unordered/unordered_map.hpp>
#include <boost/function.hpp>
#include <boost/signals.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>
#include <App/PropertyLinks.h>
#include <App/Expression.h>
#include <set>

namespace Base {
class Writer;
class XMLReader;
}

namespace App {

class DocumentObject;
class DocumentObjectExecReturn;
class ObjectIdentifier;
class Expression;


class AppExport PropertyExpressionEngine : public App::PropertyLinkBase, 
                                           private App::AtomicPropertyChangeInterface<PropertyExpressionEngine>
{
    TYPESYSTEM_HEADER();
public:

    virtual void updateElementReference(App::DocumentObject *feature,bool reverse=false) override;
    virtual bool referenceChanged() const override;
    virtual void getLinks(std::vector<App::DocumentObject *> &objs, 
            bool all=false, std::vector<std::string> *subs=0, bool newStyle=true) const override;
    virtual void breakLink(App::DocumentObject *obj, bool clear) override;
    virtual bool adjustLink(const std::set<App::DocumentObject *> &inList) override;
    virtual Property *CopyOnImportExternal(const std::map<std::string,std::string> &nameMap) const override;
    virtual Property *CopyOnLabelChange(App::DocumentObject *obj, 
                        const std::string &ref, const char *newLabel) const override;

    typedef boost::function<std::string (const App::ObjectIdentifier & path, boost::shared_ptr<const App::Expression> expr)> ValidatorFunc;
    
    /**
     * @brief The ExpressionInfo struct encapsulates an expression and a comment.
     */

    struct ExpressionInfo {
        boost::shared_ptr<App::Expression> expression; /**< The actual expression tree */
        std::string comment; /**< Optional comment for this expression */

        ExpressionInfo(boost::shared_ptr<App::Expression> expression = boost::shared_ptr<App::Expression>(), const char * comment = 0) {
            this->expression = expression;
            if (comment)
                this->comment = comment;
        }

        ExpressionInfo(const ExpressionInfo & other) {
            expression = other.expression;
            comment = other.comment;
        }

        ExpressionInfo & operator=(const ExpressionInfo & other) {
            expression = other.expression;
            comment = other.comment;
            return *this;
        }
    };

    PropertyExpressionEngine();
    ~PropertyExpressionEngine();

    unsigned int getMemSize (void) const;

    void setValue() { } // Dummy

    Property *Copy(void) const;

    void Paste(const Property &from);

    void Save (Base::Writer & writer) const;

    void Restore(Base::XMLReader &reader);

    void setValue(const App::ObjectIdentifier &path, boost::shared_ptr<App::Expression> expr, const char * comment = 0);

    const boost::any getPathValue(const App::ObjectIdentifier & path) const;

    /// Execute options
    enum ExecuteOption {
        /// Execute all expression
        ExecuteAll,
        /// Execute only output property bindings
        ExecuteOutput,
        /// Execute only non-output property bindings
        ExecuteNonOutput,
        /// Execute all expression if there are trasient property binding
        ExecuteTransient,
    };
    /** Evaluate the expressions
     * 
     * @param option: execution option, see ExecuteOption.
     */
    DocumentObjectExecReturn * execute(ExecuteOption option=ExecuteAll);

    void getDocumentObjectDeps(std::vector<DocumentObject*> & docObjs) const;

    void getPathsToDocumentObject(DocumentObject*, std::vector<App::ObjectIdentifier> & paths) const;

    bool depsAreTouched() const;

    boost::unordered_map<const App::ObjectIdentifier, const ExpressionInfo> getExpressions() const;

    /* Expression validator */
    void setValidator(ValidatorFunc f) { validator = f; }

    std::string validateExpression(const App::ObjectIdentifier & path, boost::shared_ptr<const App::Expression> expr) const;

    void renameExpressions(const std::map<App::ObjectIdentifier, App::ObjectIdentifier> &paths);

    void renameObjectIdentifiers(const std::map<App::ObjectIdentifier, App::ObjectIdentifier> & paths);

    const App::ObjectIdentifier canonicalPath(const App::ObjectIdentifier &p) const;

    size_t numExpressions() const;

    ///signal called when a expression was changed 
    boost::signal<void (const App::ObjectIdentifier &)> expressionChanged; 

    virtual void afterRestore() override;

    /* Python interface */
    PyObject *getPyObject(void);
    void setPyObject(PyObject *);

protected:
    virtual void hasSetValue() override;

private:

    typedef boost::adjacency_list< boost::listS, boost::vecS, boost::directedS > DiGraph;
    typedef std::pair<int, int> Edge;
    typedef boost::unordered_map<const App::ObjectIdentifier, ExpressionInfo> ExpressionMap;

    std::vector<App::ObjectIdentifier> computeEvaluationOrder(ExecuteOption option);

    void buildGraphStructures(const App::ObjectIdentifier &path,
                              const boost::shared_ptr<Expression> expression, boost::unordered_map<App::ObjectIdentifier, int> &nodes,
                              boost::unordered_map<int, App::ObjectIdentifier> &revNodes, std::vector<Edge> &edges) const;

    void buildGraph(const ExpressionMap &exprs,
                boost::unordered_map<int, App::ObjectIdentifier> &revNodes, 
                DiGraph &g, ExecuteOption option=ExecuteAll) const;

    bool running; /**< Boolean used to avoid loops */

    ExpressionMap expressions; /**< Stored expressions */

    ValidatorFunc validator; /**< Valdiator functor */

    struct RestoredExpression {
        std::string path;
        std::string expr;
        std::string comment;
    };
    /**< Expressions are read from file to this map first before they are validated and inserted into the actual map */
    std::unique_ptr<std::vector<RestoredExpression> > restoredExpressions;

    friend class AtomicPropertyChange;

    std::set<App::DocumentObject*> depObjs;
};

}

#endif // EXPRESSIONENGINE_H
