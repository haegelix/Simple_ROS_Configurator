class sub_pub_node {
    /**
     * Name/ID of the node.
     * Shall be unique across this package.
     * @type {string}
     */
    node_name = ""

    /**
     * Topic to interact with.
     * @type {string}
     */
    topic = ""

    /**
     * Message type. e.g. std_msgs.msg.String
     * @type {string}
     */
    type = ""

    /**
     *
     * @param {string} node_name
     * @param {string} topic
     * @param {string} type
     */
    constructor(node_name, topic, type) {
        this.node_name = node_name
        this.topic = topic
        this.type = type
    }
}

class sub extends sub_pub_node {
    /**
     * What shall be done on message received.
     * @type {string}
     */
    callback = ""

    /**
     *
     * @param {string} node_name
     * @param {string} topic
     * @param {string} type
     * @param {string} callback
     */
    constructor(node_name, topic, type, callback) {
        super(node_name, topic, type);
        this.callback = callback;
    }
}

class pub extends sub_pub_node {
    /**
     * Where shall messages come from?
     * @type {string}
     */
    src = ""

    /**
     *
     * @param {string} node_name
     * @param {string} topic
     * @param {string} type
     * @param {string} src
     */
    constructor(node_name, topic, type, src) {
        super(node_name, topic, type);
        this.src = src;
    }
}

/**
 * Holds mostly meta-data about the srosc_package.
 */
class pkg_info {
    /**
     *
     * @type {string}
     */
    description = ""
    /**
     *
     * @type {string}
     */
    license = ""
    /**
     *
     * @type {string}
     */
    version = ""
    /**
     *
     * @type {string}
     */
    maintainer = ""
    /**
     *
     * @type {string}
     */
    maintainer_mail = ""
    /**
     *
     * @type {[string]}
     */
    exec_depends = []
    /**
     *
     * @type {[string]}
     */
    test_depends = []
    constructor(package_info_json) {
        this.description = package_info_json.description
        this.license = package_info_json.license
        this.version = package_info_json.version
        this.maintainer = package_info_json.maintainer
        this.maintainer_mail = package_info_json.maintainer_mail
        this.exec_depends = package_info_json.exec_depends
        this.test_depends = package_info_json.test_depends
    }
}

/**
 * Represents the SROSC-Package structure in JS.
 */
class srosc_pkg {
    /**
     * Name of this package.
     * @type {string}
     */
    package_name = ""
    /**
     * Descriptive things.
     * @type {pkg_info}
     */
    package_info = undefined
    /**
     * Additional imports
     * @type {[string]}
     */
    additional_imports = []
    /**
     *
     * @type {[pub]}
     */
    pubs = []
    /**
     *
     * @type {[sub]}
     */
    subs = []


    /**
     * Constructs a new entity.
     * @param package_json JSON-object to be parsed.
     */
    constructor(package_json) {
        this.package_name = package_json.package_name
        this.package_info = new pkg_info(package_json.package_info)
        this.additional_imports = package_json.additional_imports
        this.pubs = this.parse_pubs(package_json.pubs)
        this.subs = this.parse_subs(package_json.subs)
    }

    /**
     * Parses the publisher-nodes from their json-object.
     * @param jsonObj
     * @returns {[pub]}
     */
    parse_pubs(jsonObj){
        const pubs = []
        for (let elem of jsonObj){
            pubs.push(new pub(elem.node_name, elem.topic, elem.type, elem.src))
        }
        return pubs
    }

    /**
     * Parses the subscriber-nodes from their json-object.
     * @param jsonObj
     * @returns {[sub]}
     */
    parse_subs(jsonObj){
        const subs = []
        for (let elem of jsonObj){
            subs.push(new pub(elem.node_name, elem.topic, elem.type, elem.callback))
        }
        return subs
    }

    /**
     * Counts the subsribers.
     * @returns {number}
     */
    count_subs() {
        return this.subs.length
    }

    /**
     * Counts the publishers.
     * @returns {number}
     */
    count_pubs() {
        return this.pubs.length
    }

    /**
     * Counts the topics in this package.
     * @returns {number}
     */
    count_topics() {
        const nodes = [...this.subs, ...this.pubs]
        const topics = []
        for (let elem of nodes) {
            if(!topics.includes(elem.topic))
                topics.push(elem.topic)
        }
        return topics.length
    }

    /**
     * Builds and returns the corresponding vis-network.
     * @returns {vis_network_data}
     */
    to_vis_network_data (){
        return package_to_network(this)
    }
}