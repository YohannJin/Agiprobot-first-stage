"""
Generate address space c++ code from xml file specification
xmlparser.py is a requirement. it is in opcua folder but to avoid importing all code, developer can link xmlparser.py in current directory
"""
import sys
import logging
# sys.path.insert(0, "..")  # load local freeopcua implementation
#from opcua import xmlparser
import xmlparser


def _to_val(objs, attr, val):
    from opcua import ua
    print("got ", objs, attr, val)
    cls = getattr(ua, objs[0])
    for o in objs[1:]:
        cls = getattr(ua, cls.ua_types[o])
    if cls == ua.NodeId:
        return "ua.NodeId.from_string('val')"
    return ua_type_to_python(val, cls.ua_types[attr])


def ua_type_to_python(val, uatype):
    if uatype in ("String"):
        return "'{0}'".format(val)
    elif uatype in ("Bytes", "Bytes", "ByteString", "ByteArray"):
        return "b'{0}'".format(val)
    else:
        return val


class CodeGenerator(object):

    def __init__(self, input_path, output_path):
        self.input_path = input_path
        self.output_path = output_path
        self.output_file = None
        self.part = self.input_path.split(".")[-2]
        self.parser = None

    def run(self):
        sys.stderr.write("Generating Python code {0} for XML file {1}".format(self.output_path, self.input_path) + "\n")
        self.output_file = open(self.output_path, "w")
        self.make_header()
        self.parser = xmlparser.XMLParser(self.input_path, None)
        for node in self.parser:
            if node.nodetype == 'UAObject':
                self.make_object_code(node)
            elif node.nodetype == 'UAObjectType':
                self.make_object_type_code(node)
            elif node.nodetype == 'UAVariable':
                self.make_variable_code(node)
            elif node.nodetype == 'UAVariableType':
                self.make_variable_type_code(node)
            elif node.nodetype == 'UAReferenceType':
                self.make_reference_code(node)
            elif node.nodetype == 'UADataType':
                self.make_datatype_code(node)
            elif node.nodetype == 'UAMethod':
                self.make_method_code(node)
            else:
                sys.stderr.write("Not implemented node type: " + node.nodetype + "\n")
        self.output_file.close()

    def writecode(self, *args):
        self.output_file.write(" ".join(args) + "\n")

    def make_header(self, ):
        self.writecode('''
"""
DO NOT EDIT THIS FILE!
It is automatically generated from opcfoundation.org schemas.
"""

from opcua import ua


def create_standard_address_space_{0!s}(server):
  '''.format((self.part)))

    def make_node_code(self, obj, indent):
        self.writecode(indent, 'node = ua.AddNodesItem()')
        self.writecode(indent, 'node.RequestedNewNodeId = ua.NodeId.from_string("{0}")'.format(obj.nodeid))
        self.writecode(indent, 'node.BrowseName = ua.QualifiedName.from_string("{0}")'.format(obj.browsename))
        self.writecode(indent, 'node.NodeClass = ua.NodeClass.{0}'.format(obj.nodetype[2:]))
        if obj.parent:
            self.writecode(indent, 'node.ParentNodeId = ua.NodeId.from_string("{0}")'.format(obj.parent))
        if obj.parent:
            self.writecode(indent, 'node.ReferenceTypeId = {0}'.format(self.to_ref_type(obj.parentlink)))
        if obj.typedef:
            self.writecode(indent, 'node.TypeDefinition = ua.NodeId.from_string("{0}")'.format(obj.typedef))

    def to_data_type(self, nodeid):
        if not nodeid:
            return "ua.NodeId(ua.ObjectIds.String)"
        if "=" in nodeid:
            return 'ua.NodeId.from_string("{0}")'.format(nodeid)
        else:
            return 'ua.NodeId(ua.ObjectIds.{0})'.format(nodeid)

    def to_ref_type(self, nodeid):
        if not "=" in nodeid:
            nodeid = self.parser.aliases[nodeid]
        return 'ua.NodeId.from_string("{0}")'.format(nodeid)

    def make_object_code(self, obj):
        indent = "   "
        self.writecode(indent)
        self.make_node_code(obj, indent)
        self.writecode(indent, 'attrs = ua.ObjectAttributes()')
        if obj.desc:
            self.writecode(indent, 'attrs.Description = ua.LocalizedText("{0}")'.format(obj.desc))
        self.writecode(indent, 'attrs.DisplayName = ua.LocalizedText("{0}")'.format(obj.displayname))
        self.writecode(indent, 'attrs.EventNotifier = {0}'.format(obj.eventnotifier))
        self.writecode(indent, 'node.NodeAttributes = attrs')
        self.writecode(indent, 'server.add_nodes([node])')
        self.make_refs_code(obj, indent)

    def make_object_type_code(self, obj):
        indent = "   "
        self.writecode(indent)
        self.make_node_code(obj, indent)
        self.writecode(indent, 'attrs = ua.ObjectTypeAttributes()')
        if obj.desc:
            self.writecode(indent, 'attrs.Description = ua.LocalizedText("{0}")'.format(obj.desc))
        self.writecode(indent, 'attrs.DisplayName = ua.LocalizedText("{0}")'.format(obj.displayname))
        self.writecode(indent, 'attrs.IsAbstract = {0}'.format(obj.abstract))
        self.writecode(indent, 'node.NodeAttributes = attrs')
        self.writecode(indent, 'server.add_nodes([node])')
        self.make_refs_code(obj, indent)

    def make_common_variable_code(self, indent, obj):
        if obj.desc:
            self.writecode(indent, 'attrs.Description = ua.LocalizedText("{0}")'.format(obj.desc))
        self.writecode(indent, 'attrs.DisplayName = ua.LocalizedText("{0}")'.format(obj.displayname))
        self.writecode(indent, 'attrs.DataType = {0}'.format(self.to_data_type(obj.datatype)))
        if obj.value is not None:
            if obj.valuetype == "ListOfExtensionObject":
                self.writecode(indent, 'value = []')
                for ext in obj.value:
                    self.make_ext_obj_code(indent, ext)
                    self.writecode(indent, 'value.append(extobj)')
                self.writecode(indent, 'attrs.Value = ua.Variant(value, ua.VariantType.ExtensionObject)')
            elif obj.valuetype == "ExtensionObject":
                self.make_ext_obj_code(indent, obj.value)
                self.writecode(indent, 'value = extobj')
                self.writecode(indent, 'attrs.Value = ua.Variant(value, ua.VariantType.ExtensionObject)')
            else:
                if obj.valuetype.startswith("ListOf"):
                    obj.valuetype = obj.valuetype[6:]
                self.writecode(indent, 'attrs.Value = ua.Variant({0}, ua.VariantType.{1})'.format(self.to_value(obj.value), obj.valuetype))
        if obj.rank:
            self.writecode(indent, 'attrs.ValueRank = {0}'.format(obj.rank))
        if obj.accesslevel:
            self.writecode(indent, 'attrs.AccessLevel = {0}'.format(obj.accesslevel))
        if obj.useraccesslevel:
            self.writecode(indent, 'attrs.UserAccessLevel = {0}'.format(obj.useraccesslevel))
        if obj.dimensions:
            self.writecode(indent, 'attrs.ArrayDimensions = {0}'.format(obj.dimensions))
    
    def make_ext_obj_code(self, indent, extobj):
        print("makeing code for ", extobj.objname)
        self.writecode(indent, 'extobj = ua.{0}()'.format(extobj.objname))
        for name, val in extobj.body.items():
            for k, v in val.items():
                if type(v) is str:
                    val = _to_val([extobj.objname], k, v)
                    self.writecode(indent, 'extobj.{0} = {1}'.format(k, val))
                else:
                    if k == "DataType":  #hack for strange nodeid xml format
                        self.writecode(indent, 'extobj.{0} = ua.NodeId.from_string("{1}")'.format(k, v["Identifier"]))
                        continue
                    for k2, v2 in v.items():
                        val2 = _to_val([extobj.objname, k], k2, v2)
                        self.writecode(indent, 'extobj.{0}.{1} = {2}'.format(k, k2, val2))

    def make_variable_code(self, obj):
        indent = "   "
        self.writecode(indent)
        self.make_node_code(obj, indent)
        self.writecode(indent, 'attrs = ua.VariableAttributes()')
        if obj.minsample:
            self.writecode(indent, 'attrs.MinimumSamplingInterval = {0}'.format(obj.minsample))
        self.make_common_variable_code(indent, obj)
        self.writecode(indent, 'node.NodeAttributes = attrs')
        self.writecode(indent, 'server.add_nodes([node])')
        self.make_refs_code(obj, indent)

    def make_variable_type_code(self, obj):
        indent = "   "
        self.writecode(indent)
        self.make_node_code(obj, indent)
        self.writecode(indent, 'attrs = ua.VariableTypeAttributes()')
        if obj.desc:
            self.writecode(indent, 'attrs.Description = ua.LocalizedText("{0}")'.format(obj.desc))
        self.writecode(indent, 'attrs.DisplayName = ua.LocalizedText("{0}")'.format(obj.displayname))
        if obj.abstract:
            self.writecode(indent, 'attrs.IsAbstract = {0}'.format(obj.abstract))
        self.make_common_variable_code(indent, obj)
        self.writecode(indent, 'node.NodeAttributes = attrs')
        self.writecode(indent, 'server.add_nodes([node])')
        self.make_refs_code(obj, indent)

    def to_value(self, val):
        # if type(val) in (str, unicode):
        if isinstance(val, str):
            return '"' + val + '"'
        else:
            return val

    def make_method_code(self, obj):
        indent = "   "
        self.writecode(indent)
        self.make_node_code(obj, indent)
        self.writecode(indent, 'attrs = ua.MethodAttributes()')
        if obj.desc:
            self.writecode(indent, 'attrs.Description = ua.LocalizedText("{0}")'.format(obj.desc))
        self.writecode(indent, 'attrs.DisplayName = ua.LocalizedText("{0}")'.format(obj.displayname))
        self.writecode(indent, 'node.NodeAttributes = attrs')
        self.writecode(indent, 'server.add_nodes([node])')
        self.make_refs_code(obj, indent)

    def make_reference_code(self, obj):
        indent = "   "
        self.writecode(indent)
        self.make_node_code(obj, indent)
        self.writecode(indent, 'attrs = ua.ReferenceTypeAttributes()')
        if obj.desc:
            self.writecode(indent, 'attrs.Description = ua.LocalizedText("{0}")'.format(obj.desc))
        self.writecode(indent, 'attrs.DisplayName = ua.LocalizedText("{0}")'.format(obj.displayname))
        if obj. inversename:
            self.writecode(indent, 'attrs.InverseName = ua.LocalizedText("{0}")'.format(obj.inversename))
        if obj.abstract:
            self.writecode(indent, 'attrs.IsAbstract = {0}'.format(obj.abstract))
        if obj.symmetric:
            self.writecode(indent, 'attrs.Symmetric = {0}'.format(obj.symmetric))
        self.writecode(indent, 'node.NodeAttributes = attrs')
        self.writecode(indent, 'server.add_nodes([node])')
        self.make_refs_code(obj, indent)

    def make_datatype_code(self, obj):
        indent = "   "
        self.writecode(indent)
        self.make_node_code(obj, indent)
        self.writecode(indent, 'attrs = ua.DataTypeAttributes()')
        if obj.desc:
            self.writecode(indent, u'attrs.Description = ua.LocalizedText("{0}")'.format(obj.desc.encode('ascii', 'replace')))
        self.writecode(indent, 'attrs.DisplayName = ua.LocalizedText("{0}")'.format(obj.displayname))
        if obj.abstract:
            self.writecode(indent, 'attrs.IsAbstract = {0}'.format(obj.abstract))
        self.writecode(indent, 'node.NodeAttributes = attrs')
        self.writecode(indent, 'server.add_nodes([node])')
        self.make_refs_code(obj, indent)

    def make_refs_code(self, obj, indent):
        if not obj.refs:
            return
        self.writecode(indent, "refs = []")
        for ref in obj.refs:
            self.writecode(indent, 'ref = ua.AddReferencesItem()')
            self.writecode(indent, 'ref.IsForward = True')
            self.writecode(indent, 'ref.ReferenceTypeId = {0}'.format(self.to_ref_type(ref.reftype)))
            self.writecode(indent, 'ref.SourceNodeId = ua.NodeId.from_string("{0}")'.format(obj.nodeid))
            self.writecode(indent, 'ref.TargetNodeClass = ua.NodeClass.DataType')
            self.writecode(indent, 'ref.TargetNodeId = ua.NodeId.from_string("{0}")'.format(ref.target))
            self.writecode(indent, "refs.append(ref)")
        self.writecode(indent, 'server.add_references(refs)')


def save_aspace_to_disk():
    import os.path
    path = os.path.join("..", "opcua", "binary_address_space.pickle")
    print("Savind standard address space to:", path)
    sys.path.append("..")
    from opcua.server.standard_address_space import standard_address_space
    from opcua.server.address_space import NodeManagementService, AddressSpace
    aspace = AddressSpace()
    standard_address_space.fill_address_space(NodeManagementService(aspace))
    aspace.dump(path)

if __name__ == "__main__":
    logging.basicConfig(level=logging.WARN)
    for i in (3, 4, 5, 8, 9, 10, 11, 13):
        xmlpath = "Opc.Ua.NodeSet2.Part{0}.xml".format(str(i))
        cpppath = "../opcua/server/standard_address_space/standard_address_space_part{0}.py".format(str(i))
        c = CodeGenerator(xmlpath, cpppath)
        c.run()

    save_aspace_to_disk()
