// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/* Copyright 2023 NXP */
#include "llce_can_core.h"

#define PATH_DIR(P, D)			P "/" D

#define DEBUGFS_ROOT			"/sys/kernel/debug"

#define LLCE_CAN_CORE_DIR		"llce_can_core"
#define LLCE_CAN_CORE_PATH		PATH_DIR(DEBUGFS_ROOT, LLCE_CAN_CORE_DIR)

#define LLCE_CAN_DEST_DIR		"can_destinations"
#define ADD_CMD_DIR			"add"
#define README_FILE			"README"
#define LIST_FILE			"list"
#define EXECUTE_FILE			"execute"

#define LLCE_CAN_DEST_RPATH		PATH_DIR(LLCE_CAN_CORE_PATH,\
						 LLCE_CAN_DEST_DIR)
#define LLCE_CAN_DEST_PATH(D)		PATH_DIR(LLCE_CAN_DEST_RPATH, D)
#define ADD_DEST_CMD_PATH(D)		PATH_DIR(LLCE_CAN_DEST_PATH(ADD_CMD_DIR), D)

#define DEST_ID_FLD			"id"
#define DEST_ROUTING_OPS_FLD		"routing_opts"
#define DEST_CAN_ID_REMAP_FLD		"can_id_remap"
#define DEST_CH_LIST_FLD		"ch_list"

struct u8_array_meta {
	struct debugfs_u8_array *array;
	size_t alloc_size;
	char *buffer;
};

struct debugfs_dir_create {
	struct dentry **parent, **dir;
	const char *name;
	char **readme;
};

/* Tentative definitions for README files content */
static char *root_readme;
static char *can_dest_readme;
static char *can_dest_add_cmd_readme;

static void debugfs_create_readme(struct dentry *parent, char **value)
{
	debugfs_create_str(README_FILE, 0400, parent, value);
}

static int debugfs_cmd_set(void *data, u64 val)
{
	struct debugfs_cmd_block *blk = data;

	if (!blk || !blk->clb)
		return -EINVAL;

	return blk->clb(blk, val);
}

static int debugfs_cmd_get(void *data, u64 *val)
{
	struct debugfs_cmd_block *blk = data;

	if (!blk || !blk->get_val)
		return -EINVAL;

	return blk->get_val(blk, val);
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_cmd, debugfs_cmd_get, debugfs_cmd_set, "%llu\n");

static int debugfs_create_cmd(struct device *dev, const char *name,
			      struct dentry *parent,
			      struct debugfs_cmd_block *blk)
{
	struct dentry *file;

	file = debugfs_create_file_unsafe(name, 0600,
					  parent, blk, &fops_cmd);
	if (IS_ERR(file)) {
		dev_err(dev, "Failed to create the file '%s'\n", name);
		return PTR_ERR(file);
	}

	return 0;
}

static int u8_array_open(struct inode *inode, struct file *file)
{
	struct debugfs_u8_array *data = inode->i_private;
	struct u8_array_meta *meta;
	int ret;

	meta = kmalloc(sizeof(*meta), GFP_KERNEL);
	if (!meta)
		return -ENOMEM;

	meta->array = data;

	/* Max size:
	 *  - 10 bytes per number
	 *  - terminating NUL character
	 */
	meta->alloc_size = data->max_elems * 10 + 1;
	meta->buffer = kmalloc(meta->alloc_size, GFP_KERNEL);
	if (!meta->buffer) {
		ret = -ENOMEM;
		goto release_meta;
	}

	meta->buffer[meta->alloc_size - 1] = 0;

	file->private_data = meta;

	ret = nonseekable_open(inode, file);
	if (ret)
		kfree(meta->buffer);

release_meta:
	if (ret)
		kfree(meta);

	return ret;
}

static int u8_array_release(struct inode *inode, struct file *file)
{
	struct u8_array_meta *meta = file->private_data;

	kfree(meta->buffer);
	kfree(meta);

	return 0;
}

static ssize_t u8_array_read(struct file *file, char __user *buf, size_t len,
			     loff_t *ppos)
{
	struct u8_array_meta *meta = file->private_data;
	struct debugfs_u8_array *data = meta->array;
	u8 *array = data->array;
	u8 n_elements = data->n_elements;
	size_t size, bufsize;
	char *str, term;
	int elem_len;

	bufsize = meta->alloc_size;

	meta->buffer[0] = 0;
	str = meta->buffer;

	while (n_elements > 0) {
		term = (n_elements > 1) ? ' ' : '\n';

		elem_len = snprintf(str, bufsize, "%hhu%c", *array++, term);
		if (elem_len < 0) {
			pr_err("Failed to read an array member from string: '%s'\n",
			       str);
			return elem_len;
		}

		if (bufsize < elem_len)
			return -EINVAL;

		str += elem_len;
		bufsize -= elem_len;
		n_elements--;
	}

	size = str - meta->buffer;

	return simple_read_from_buffer(buf, len, ppos, meta->buffer, size);
}

static ssize_t u8_array_write(struct file *file, const char __user *user_buf,
			      size_t count, loff_t *ppos)
{
	struct u8_array_meta *meta = file->private_data;
	struct debugfs_u8_array *data = meta->array;
	u8 *array = data->array;
	int pos;
	char *buf;
	ssize_t ret;

	if (count > meta->alloc_size)
		return -E2BIG;

	data->n_elements = 0;

	ret = simple_write_to_buffer(meta->buffer, meta->alloc_size, ppos,
				     user_buf, count);
	if (ret < 0)
		return ret;

	buf = meta->buffer;
	while (sscanf(buf, "%hhu%n", array, &pos) == 1) {
		if (data->n_elements >= data->max_elems)
			return -E2BIG;

		array++;
		data->n_elements++;
		buf += pos;
	}

	return ret;
}

static const struct file_operations u8_array_fops = {
	.owner	 = THIS_MODULE,
	.open	 = u8_array_open,
	.release = u8_array_release,
	.read	 = u8_array_read,
	.write	 = u8_array_write,
	.llseek	 = no_llseek,
};

static int debugfs_create_u8_array(struct device *dev, const char *name,
				   struct dentry *parent,
				   struct debugfs_u8_array *array)
{
	struct dentry *file;

	file = debugfs_create_file_unsafe(name, 0600, parent, array,
					  &u8_array_fops);
	if (IS_ERR(file)) {
		dev_err(dev, "Failed to create the file '%s'\n", name);
		return PTR_ERR(file);
	}

	return 0;
}

static struct llce_can_core *get_can_core(struct llce_can_core_debugfs *root)
{
	return container_of(root, struct llce_can_core, debugfs);
}

static struct device *get_llce_dbgfs_device(struct llce_can_core_debugfs *root)
{
	struct llce_can_core *core = get_can_core(root);

	return get_can_core_dev(core);
}

static void *dest_list_seq_start(struct seq_file *seq, loff_t *pos)
{
	struct llce_can_core *can_core = seq->private;

	mutex_lock(&can_core->can_dest_lock);
	return seq_list_start_head(&can_core->can_dest_list, *pos);
}

static void *dest_list_seq_next(struct seq_file *seq, void *v, loff_t *pos)
{
	struct llce_can_core *can_core = seq->private;

	return seq_list_next(v, &can_core->can_dest_list, pos);
}

static void dest_list_seq_stop(struct seq_file *seq, void *v)
{
	struct llce_can_core *can_core = seq->private;

	mutex_unlock(&can_core->can_dest_lock);
}

static int dest_list_seq_show(struct seq_file *seq, void *v)
{
	const struct can_destination *can_dest;
	struct llce_can_core *can_core = seq->private;
	u8 idx;

	can_dest = list_entry(v, struct can_destination, link);

	if (v == &can_core->can_dest_list) {
		seq_printf(seq, "%3s %s %s %s\n",
			   DEST_ID_FLD, DEST_ROUTING_OPS_FLD,
			   DEST_CAN_ID_REMAP_FLD, DEST_CH_LIST_FLD);
		return 0;
	}

	seq_printf(seq, "%3hhu   0x%08x   0x%08x", can_dest->id,
		   can_dest->dest.can2can_routing_options,
		   can_dest->dest.can_id_remap_value);

	for (idx = 0; idx < can_dest->dest.dest_hw_ch_list_count; idx++)
		seq_printf(seq, " %hhu", can_dest->dest.dest_hw_ch_list[idx]);

	seq_puts(seq, "\n");

	return 0;
}

static const struct seq_operations dest_list_seq_ops = {
	.start = dest_list_seq_start,
	.next  = dest_list_seq_next,
	.stop  = dest_list_seq_stop,
	.show  = dest_list_seq_show,
};

struct list_files_entry {
	const char *parent_name;
	const struct seq_operations *seq_ops;
};

static struct list_files_entry list_map[] = {
	{
		.parent_name = LLCE_CAN_DEST_DIR,
		.seq_ops = &dest_list_seq_ops,
	},
};

static int can_core_list_open(struct inode *inode, struct file *file)
{
	const struct seq_operations *seq_ops = NULL;
	const char *folder_name = (const char *)file->f_path.dentry->d_parent->d_iname;
	void *data = inode->i_private;
	struct seq_file *seq;
	size_t i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(list_map); i++) {
		if (strcmp(folder_name, list_map[i].parent_name))
			continue;

		seq_ops = list_map[i].seq_ops;
		break;
	}

	if (!seq_ops)
		return -EINVAL;

	ret = seq_open(file, seq_ops);
	if (!ret) {
		seq = file->private_data;
		seq->private = data;
	}

	return ret;
}

static int can_core_list_release(struct inode *inode, struct file *file)
{
	return seq_release(inode, file);
}

static const struct file_operations can_core_list_debugfs_fops = {
	.open		= can_core_list_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= can_core_list_release,
};

void llce_remove_debugs(struct llce_can_core_debugfs *root)
{
	debugfs_remove_recursive(root->root_dir);
}

static int can_get_last_dest_id(void *data, u64 *val)
{
	struct debugfs_cmd_block *blk = data;
	struct llce_can_dest_debugfs *can_dest;

	can_dest = container_of(blk, struct llce_can_dest_debugfs, add_exec);

	*val = can_dest->id;

	return 0;
}

static int validate_can_add_dest_params(struct device *dev,
					struct llce_can_dest_debugfs *can_dest)
{
	struct debugfs_u8_array *channels;
	u8 i;

	channels = &can_dest->ch_list;

	if (channels->n_elements > LLCE_CAN_CONFIG_MAXCTRL_COUNT) {
		dev_err(dev, "The number of channels exceeds the allowed limit of %u elements\n",
			LLCE_CAN_CONFIG_MAXCTRL_COUNT);
		return -ENOSPC;
	}

	for (i = 0; i < channels->n_elements; i++) {
		if (channels->array[i] < LLCE_CAN_CONFIG_MAXCTRL_COUNT)
			continue;

		dev_err(dev, "Channel id %u is invalid\n",
			channels->array[i]);

		return -EINVAL;
	}

	return 0;
}

static int can_add_dest_func(void *data, u64 val)
{
	struct debugfs_cmd_block *blk = data;
	struct llce_can_can2can_routing_table route_dest;
	struct llce_can_dest_debugfs *can_dest;
	struct llce_can_core_debugfs *core_debugfs;
	struct llce_can_core *can_core;
	struct device *dev;
	int ret;

	can_dest = container_of(blk, struct llce_can_dest_debugfs, add_exec);
	core_debugfs = container_of(can_dest, struct llce_can_core_debugfs,
				    can_dest);
	can_core = get_can_core(core_debugfs);
	dev = get_llce_dbgfs_device(core_debugfs);

	route_dest = (struct llce_can_can2can_routing_table) {
		.can2can_routing_options = can_dest->routing_opts,
		.can_id_remap_value = can_dest->can_id_remap,
		.dest_hw_ch_list_count = can_dest->ch_list.n_elements,
	};

	ret = validate_can_add_dest_params(dev, can_dest);
	if (ret)
		return ret;

	if (route_dest.dest_hw_ch_list_count >
	    ARRAY_SIZE(route_dest.dest_hw_ch_list)) {
		dev_err(dev, "Too many channels\n");
		return -ENOSPC;
	}

	memcpy(&route_dest.dest_hw_ch_list,
	       can_dest->ch_list.array,
	       sizeof(route_dest.dest_hw_ch_list[0]) *
	       route_dest.dest_hw_ch_list_count);

	ret = llce_add_can_dest(can_core, &route_dest, &can_dest->id);
	if (ret)
		dev_err(dev, "Failed to add a CAN2CAN route\n");

	return ret;
}

static int debugfs_create_dirs(struct device *dev,
			       struct debugfs_dir_create *dirs, size_t n_dirs)
{
	struct dentry *folder;
	struct dentry **ptr_folder;
	size_t i;

	for (i = 0; i < n_dirs; i++) {
		if (!dirs[i].dir)
			ptr_folder = &folder;
		else
			ptr_folder = dirs[i].dir;

		*ptr_folder = debugfs_create_dir(dirs[i].name, *dirs[i].parent);
		if (IS_ERR(*ptr_folder)) {
			dev_err(dev, "Failed to create %s directory\n",
				dirs[i].name);
			return PTR_ERR(*ptr_folder);
		}

		debugfs_create_readme(*ptr_folder, dirs[i].readme);
	}

	return 0;
}

static int debugfs_create_list(struct llce_can_core *can_core,
			       struct dentry *parent)
{
	struct dentry *list_file;

	list_file = debugfs_create_file(LIST_FILE, 0400, parent, can_core,
					&can_core_list_debugfs_fops);
	if (IS_ERR(list_file))
		return PTR_ERR(list_file);

	return 0;
}

static int create_can_dest_tree(struct llce_can_core_debugfs *root)
{
	struct dentry *add_folder, *dest_folder;
	struct device *dev = get_llce_dbgfs_device(root);
	struct llce_can_core *can_core = get_can_core(root);
	struct llce_can_dest_debugfs *can_dest = &root->can_dest;
	struct debugfs_dir_create folders[] = {
		{
			.parent = &root->root_dir,
			.name = LLCE_CAN_DEST_DIR,
			.dir = &dest_folder,
			.readme = &can_dest_readme
		},
		{
			.parent = &dest_folder,
			.name = ADD_CMD_DIR,
			.dir = &add_folder,
			.readme = &can_dest_add_cmd_readme,
		},
	};
	int ret;

	ret = debugfs_create_dirs(dev, folders, ARRAY_SIZE(folders));
	if (ret)
		return ret;

	can_dest->ch_list.array = &can_dest->dest_hw_ch_list[0];
	can_dest->ch_list.max_elems = ARRAY_SIZE(can_dest->dest_hw_ch_list);
	can_dest->ch_list.n_elements = 0;

	ret = debugfs_create_list(can_core, dest_folder);
	if (ret)
		return ret;

	debugfs_create_x32(DEST_ROUTING_OPS_FLD, 0600,
			   add_folder, &can_dest->routing_opts);

	debugfs_create_x32(DEST_CAN_ID_REMAP_FLD, 0600,
			   add_folder,
			   &can_dest->can_id_remap);

	ret = debugfs_create_u8_array(dev, DEST_CH_LIST_FLD, add_folder,
				      &can_dest->ch_list);
	if (ret)
		return ret;

	can_dest->add_exec.clb = can_add_dest_func;
	can_dest->add_exec.get_val = can_get_last_dest_id;

	ret = debugfs_create_cmd(dev, EXECUTE_FILE, add_folder,
				 &can_dest->add_exec);
	if (ret)
		return ret;

	return 0;
}

int llce_create_debugfs(struct llce_can_core_debugfs *root)
{
	struct device *dev = get_llce_dbgfs_device(root);
	int ret;

	root->root_dir = debugfs_create_dir(LLCE_CAN_CORE_DIR, NULL);
	if (IS_ERR(root->root_dir)) {
		dev_err(dev, "Failed to create %s directory\n",
			LLCE_CAN_CORE_DIR);
		return PTR_ERR(root->root_dir);
	}

	debugfs_create_readme(root->root_dir, &root_readme);

	ret = create_can_dest_tree(root);
	if (ret)
		goto remove_debugfs_root;

remove_debugfs_root:
	if (ret)
		llce_remove_debugs(root);

	return ret;
}

static char *root_readme = "\n"
"This folder contains various configuration files of the LLCE CAN subsystem\n"
"that do not have an alternative in classic SocketCAN tools. Each functionality\n"
"is placed into a subfolder to isolate the context and ease navigation. README\n"
"files can be found in each directory to provide more details about the context\n"
"and usage of the offered settings.\n"
"\n"
"This is a summary of the subfolders:\n"
"- " LLCE_CAN_DEST_DIR "\n"
"    Manages CAN2CAN destinations used for frames accepted by LLCE CAN advanced\n"
"    filters.\n"
"\n"
"Some subdirectories are responsible for various LLCE CAN commands. These"
"sub-folders contain field files whose format may vary from file to file. The\n"
"content format can be found by inspecting the README in the same folder as the\n"
"field or by checking the file contents.\n";

static char *can_dest_readme = "\n"
"The LLCE CAN destinations are rules to be used for frames accepted by LLCE CAN\n"
"advanced filters. These rules can be either added to the firmware's table or\n"
"inspected. The formation of a new destination is managed by files from '"
ADD_CMD_DIR "'\n"
"subfolder. After successfully creating the destination, it may be attached to\n"
"one or more advanced filters. Finally, the listing of all added rules can be\n"
"performed by inspecting the content of the '" LIST_FILE "' file:\n"
"    cat " LLCE_CAN_DEST_PATH(LIST_FILE) "\n";

static char *can_dest_add_cmd_readme = "\n"
"This folder contains files designated for LLCE CAN destinations creation.\n"
"\n"
"The '" ADD_CMD_DIR "' command inserts a new entry into the LLCE firmware destinations table\n"
"for CAN2CAN routes. It is a wrapper for the LLCE firmware's\n"
"LLCE_CAN_CMD_CREATE_AF_DESTINATION command and requires three parameters:\n"
"    - '" DEST_ROUTING_OPS_FLD "'\n"
"    - '" DEST_CAN_ID_REMAP_FLD "'\n"
"    - '" DEST_CH_LIST_FLD "'\n"
"\n"
"Each parameter will be written into its dedicated file, part of this folder.\n"
"The files mentioned above keep the state of a destination that was not submitted\n"
"yet. The content of the field files will be interpreted and the command will be\n"
"sent to the firmware after a write into '" EXECUTE_FILE "'.\n"
"\n"
"An example showing its use:\n"
"    echo '0x1' > " ADD_DEST_CMD_PATH(DEST_ROUTING_OPS_FLD) "\n"
"    echo '1 2 3 4' > " ADD_DEST_CMD_PATH(DEST_CH_LIST_FLD) "\n"
"    echo '1' > " ADD_DEST_CMD_PATH(EXECUTE_FILE) "\n"
"\n"
"    The firmware returns a rule index in case the destination was successfully\n"
"    added. It can be inspected by dumping the content of the '" EXECUTE_FILE "' file:\n"
"    cat "  ADD_DEST_CMD_PATH(EXECUTE_FILE) "\n"
"\n"
"More details about each parameter can be found in the LLCE Firmware User Guide.\n"
"Here are some references that can be used as a starting point.\n"
"\n"
"......................................................\n"
":  Parameter   :          LLCE FW reference          :\n"
":..............:.....................................:\n"
": " DEST_ROUTING_OPS_FLD " : u32Can2CanRoutingOptions            :\n"
": " DEST_CAN_ID_REMAP_FLD " : u32CanIdRemapValue                  :\n"
": " DEST_CH_LIST_FLD "      : u8DestHwChList, u8DestHwChListCount :\n"
":..............:.....................................:\n";

