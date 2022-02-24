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
    msg_type = ""

    /**
     * Does this node write messages to stdout?
     * @type {boolean}
     */
    uses_stdout = false

    /**
     * Does this node need tty-emulation?
     * @type {boolean}
     */
    needs_tty = false

    /**
     * Constructor.
     * @param jsonObj
     */
    constructor(jsonObj) {
        this.node_name = jsonObj.node_name
        this.topic = jsonObj.topic
        this.msg_type = jsonObj.msg_type
        this.uses_stdout = jsonObj.uses_stdout
        this.needs_tty = jsonObj.needs_tty
    }
}

class sub extends sub_pub_node {
    /**
     * What shall be done on message received.
     * @type {string}
     */
    callback = ""
    /**
     * Custom code.
     * @type {string}
     */
    user_code = ""

    /**
     *
     * @param jsonObj
     */
    constructor(jsonObj) {
        super(jsonObj);
        this.callback = jsonObj.callback;
        this.user_code = jsonObj.user_code;
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
     * @param jsonObj
     */
    constructor(jsonObj) {
        super(jsonObj);
        this.src = jsonObj.src;
    }
}

class ros2_ui_project_dependencies {
    /**
     *
     * @type {[string]}
     */
    test_depends = []
    /**
     *
     * @type {[string]}
     */
    exec_depends = []

    constructor(project_dependencies_json) {
        this.test_depends = project_dependencies_json.test_depends
        this.exec_depends = project_dependencies_json.exec_depends
    }
}

/**
 * Holds mostly meta-data about the ros2_ui.
 */
class ros2_ui_project_info {
    /**
     *
     * @type {string}
     */
    package_name = ""
    /**
     *
     * @type {string}
     */
    version = ""
    /**
     *
     * @type {string}
     */
    description = ""
    /**
     *
     * @type {string}
     */
    maintainer_mail = ""
    /**
     *
     * @type {string}
     */
    maintainer = ""
    /**
     *
     * @type {string}
     */
    license = ""

    constructor(project_info_json) {
        this.package_name = project_info_json.package_name
        this.version = project_info_json.version
        this.description = project_info_json.description
        this.maintainer_mail = project_info_json.maintainer_mail
        this.maintainer = project_info_json.maintainer
        this.license = project_info_json.license
    }
}

/**
 * Represents the ros2_ui-project structure in JS.
 */
class ros2_ui_project {
    /**
     * Descriptive things.
     * @type {ros2_ui_project_info}
     */
    project_info = undefined
    /**
     *
     * @type {ros2_ui_project_dependencies}
     */
    project_dependencies = undefined
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
     * @param project_json JSON-object to be parsed.
     */
    constructor(project_json) {
        this.project_info = new ros2_ui_project_info(project_json.project_info)
        this.project_dependencies = new ros2_ui_project_dependencies(project_json.project_dependencies)
        this.pubs = this.parse_pubs(project_json.pubs)
        this.subs = this.parse_subs(project_json.subs)
    }

    /**
     * Parses the publisher-nodes from their json-object.
     * @param jsonObj
     * @returns {[pub]}
     */
    parse_pubs(jsonObj) {
        const pubs = []
        for (let elem of jsonObj) {
            pubs.push(new pub(elem))
        }
        return pubs
    }

    /**
     * Parses the subscriber-nodes from their json-object.
     * @param jsonObj
     * @returns {[sub]}
     */
    parse_subs(jsonObj) {
        const subs = []
        for (let elem of jsonObj) {
            subs.push(new sub(elem))
        }
        return subs
    }

    /**
     * Counts the subscribers.
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
            if (!topics.includes(elem.topic)) topics.push(elem.topic)
        }
        return topics.length
    }

    /**
     * Remove a publisher.
     * @param node_name {string}
     */
    remove_pub(node_name) {
        this.pubs = this.pubs.filter(function (e) {
            return e.node_name !== node_name
        })
    }

    /**
     * Remove a subscriber.
     * @param node_name {string}
     */
    remove_sub(node_name) {
        this.subs = this.subs.filter(function (e) {
            return e.node_name !== node_name
        })
    }

    /**
     *
     * @param id {string}
     * @returns {sub_pub_node}
     */
    get_node_by_node_name(id) {
        const nodes = [...this.subs, ...this.pubs]
        for (let elem of nodes) {
            if (elem.node_name === id) {
                return elem
            }
        }
    }

    /**
     *
     * @param node {sub_pub_node}
     */
    add_node(node) {
        if (node instanceof pub) {
            this.remove_pub(node.node_name)
            this.pubs.push(node)
        } else if (node instanceof sub) {
            this.remove_sub(node.node_name)
            this.subs.push(node)
        }
    }
}

/**
 * Returns an empty project.
 * @return {ros2_ui_project}
 */
function get_empty_ros2ui_project() {
    return new ros2_ui_project({
        project_info: {
            package_name: "test",
            version: "",
            description: "",
            maintainer_mail: "",
            maintainer: "",
            license: "",
        },
        project_dependencies: {
            test_depends: [],
            exec_depends: []
        }, pubs: [], subs: []
    })
}