/*
 *  yosys -- Yosys Open SYnthesis Suite
 *
 *  Copyright (C) 2021  Marcelina Kościelnicka <mwk@0x04.net>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include <functional>
#include <algorithm>

#include "kernel/yosys.h"
#include "kernel/sigtools.h"
#include "kernel/mem.h"
#include "kernel/qcsat.h"

USING_YOSYS_NAMESPACE
PRIVATE_NAMESPACE_BEGIN

#define FACTOR_MUX 0.5
#define FACTOR_DEMUX 0.5
#define FACTOR_EMU 2

struct PassOptions {
	bool no_auto_distributed;
	bool no_auto_block;
	bool no_auto_huge;
	bool debug_geom;
};

enum class RamKind {
	Auto,
	Logic,
	NotLogic,
	Distributed,
	Block,
	Huge,
};

enum class MemoryInitKind {
	None,
	Zero,
	Any,
};

enum class PortKind {
	Sr,
	Ar,
	Sw,
	Srsw,
	Arsw,
};

enum class ClkPolKind {
	Anyedge,
	Posedge,
	Negedge,
};

enum class RdEnKind {
	None,
	Any,
	WriteImplies,
	WriteExcludes,
};

enum class ResetKind {
	Init,
	Async,
	Sync,
};

enum class ResetValKind {
	None,
	Zero,
	Named,
};

enum class SrstKind {
	SrstOverEn,
	EnOverSrst,
	Any,
};

enum class TransTargetKind {
	Self,
	Other,
	Named,
};

enum class TransKind {
	New,
	Old,
};

typedef dict<std::string, Const> Options;

struct Empty {};

struct ClockDef {
	ClkPolKind kind;
	std::string name;
};

struct ResetValDef {
	ResetKind kind;
	ResetValKind val_kind;
	std::string name;
};

struct WrTransDef {
	TransTargetKind target_kind;
	std::string target_name;
	TransKind kind;
};

struct WidthDef {
	bool tied;
	std::vector<int> wr_widths;
	std::vector<int> rd_widths;
};

template<typename T> struct Capability {
	T val;
	Options opts, portopts;

	Capability(T val, Options opts, Options portopts) : val(val), opts(opts), portopts(portopts) {}
};

template<typename T> using Caps = std::vector<Capability<T>>;

struct PortGroupDef {
	PortKind kind;
	std::vector<std::string> names;
	Caps<ClockDef> clock;
	Caps<WidthDef> width;
	Caps<Empty> addrce;
	Caps<RdEnKind> rden;
	Caps<ResetValDef> rdrstval;
	Caps<SrstKind> rdsrstmode;
	Caps<std::string> wrprio;
	Caps<WrTransDef> wrtrans;
	Caps<int> wrcs;
};

struct MemoryDimsDef {
	int abits;
	std::vector<int> dbits;
	bool tied;
	std::string resource_name;
	int resource_count;
	double cost;
};

struct RamDef {
	IdString id;
	RamKind kind;
	bool prune_rom;
	Caps<PortGroupDef> ports;
	Caps<MemoryDimsDef> dims;
	Caps<int> byte;
	Caps<MemoryInitKind> init;
	Caps<std::string> style;
};

struct Library {
	std::vector<RamDef> ram_defs;
	PassOptions opts;
	const pool<std::string> defines;
	pool<std::string> defines_unused;
	dict<std::string, double> costs;

	Library(PassOptions opts, pool<std::string> defines) : opts(opts), defines(defines), defines_unused(defines) {}

	void prepare() {
		for (auto def: defines_unused) {
			log_warning("define %s not used in the library.\n", def.c_str());
		}
	}
};

bool opts_conflict(const Options &a, const Options &b) {
	for (auto &it: a) {
		auto it2 = b.find(it.first);
		if (it2 != b.end() && it.second != it2->second)
			return true;
	}
	return false;
}

struct Parser {
	std::string filename;
	std::ifstream infile;
	int line_number = 0;
	Library &lib;
	std::vector<std::string> tokens;
	int token_idx = 0;
	bool eof = false;

	std::vector<std::pair<std::string, Const>> option_stack;
	std::vector<std::pair<std::string, Const>> portoption_stack;
	RamDef ram;
	PortGroupDef port;
	bool active = true;

	Parser(std::string filename, Library &lib) : filename(filename), lib(lib) {
		// Note: this rewrites the filename we're opening, but not
		// the one we're storing — this is actually correct, so that
		// we keep the original filename for diagnostics.
		rewrite_filename(filename);
		infile.open(filename);
		if (infile.fail()) {
			log_error("failed to open %s\n", filename.c_str());
		}
		parse();
		infile.close();
	}

	std::string peek_token() {
		if (eof)
			return "";

		if (token_idx < GetSize(tokens))
			return tokens[token_idx];

		tokens.clear();
		token_idx = 0;

		std::string line;
		while (std::getline(infile, line)) {
			line_number++;
			for (string tok = next_token(line); !tok.empty(); tok = next_token(line)) {
				if (tok[0] == '#')
					break;
				if (tok[tok.size()-1] == ';') {
					tokens.push_back(tok.substr(0, tok.size()-1));
					tokens.push_back(";");
				} else {
					tokens.push_back(tok);
				}
			}
			if (!tokens.empty())
				return tokens[token_idx];
		}

		eof = true;
		return "";
	}

	std::string get_token() {
		std::string res = peek_token();
		if (!eof)
			token_idx++;
		return res;
	}

	void eat_token(std::string expected) {
		std::string token = get_token();
		if (token != expected) {
			log_error("%s:%d: expected `%s`, got `%s`.\n", filename.c_str(), line_number, expected.c_str(), token.c_str());
		}
	}

	IdString get_id() {
		std::string token = get_token();
		if (token.empty() || (token[0] != '$' && token[0] != '\\')) {
			log_error("%s:%d: expected id string, got `%s`.\n", filename.c_str(), line_number, token.c_str());
		}
		return IdString(token);
	}

	std::string get_name() {
		std::string res = get_token();
		bool valid = true;
		// Basic sanity check.
		if (res.empty() || (!isalpha(res[0]) && res[0] != '_'))
			valid = false;
		for (char c: res)
			if (!isalnum(c) && c != '_')
				valid = false;
		if (!valid)
			log_error("%s:%d: expected name, got `%s`.\n", filename.c_str(), line_number, res.c_str());
		return res;
	}

	std::string get_string() {
		std::string token = get_token();
		if (token.size() < 2 || token[0] != '"' || token[token.size()-1] != '"') {
			log_error("%s:%d: expected string, got `%s`.\n", filename.c_str(), line_number, token.c_str());
		}
		return token.substr(1, token.size()-2);
	}

	bool peek_string() {
		std::string token = peek_token();
		return !token.empty() && token[0] == '"';
	}

	int get_int() {
		std::string token = get_token();
		char *endptr;
		long res = strtol(token.c_str(), &endptr, 0);
		if (token.empty() || *endptr || res > INT_MAX) {
			log_error("%s:%d: expected int, got `%s`.\n", filename.c_str(), line_number, token.c_str());
		}
		return res;
	}

	double get_double() {
		std::string token = get_token();
		char *endptr;
		double res = strtod(token.c_str(), &endptr);
		if (token.empty() || *endptr) {
			log_error("%s:%d: expected float, got `%s`.\n", filename.c_str(), line_number, token.c_str());
		}
		return res;
	}

	bool peek_int() {
		std::string token = peek_token();
		return !token.empty() && isdigit(token[0]);
	}

	void get_semi() {
		std::string token = get_token();
		if (token != ";") {
			log_error("%s:%d: expected `;`, got `%s`.\n", filename.c_str(), line_number, token.c_str());
		}
	}

	Const get_value() {
		std::string token = peek_token();
		if (!token.empty() && token[0] == '"') {
			std::string s = get_string();
			return Const(s);
		} else {
			return Const(get_int());
		}
	}

	bool enter_ifdef(bool polarity) {
		bool res = active;
		std::string name = get_name();
		lib.defines_unused.erase(name);
		if (lib.defines.count(name)) {
			active = polarity;
		} else {
			active = !polarity;
		}
		return res;
	}

	void enter_else(bool save) {
		get_token();
		active = !active && save;
	}

	void enter_option() {
		std::string name = get_string();
		Const val = get_value();
		option_stack.push_back({name, val});
	}

	void exit_option() {
		option_stack.pop_back();
	}

	Options get_options() {
		Options res;
		for (auto it: option_stack)
			res[it.first] = it.second;
		return res;
	}

	void enter_portoption() {
		std::string name = get_string();
		Const val = get_value();
		portoption_stack.push_back({name, val});
	}

	void exit_portoption() {
		portoption_stack.pop_back();
	}

	Options get_portoptions() {
		Options res;
		for (auto it: portoption_stack)
			res[it.first] = it.second;
		return res;
	}

	template<typename T> void add_cap(Caps<T> &caps, T val) {
		if (active)
			caps.push_back(Capability<T>(val, get_options(), get_portoptions()));
	}

	void parse_port_block() {
		if (peek_token() == "{") {
			get_token();
			while (peek_token() != "}")
				parse_port_item();
			get_token();
		} else {
			parse_port_item();
		}
	}

	void parse_ram_block() {
		if (peek_token() == "{") {
			get_token();
			while (peek_token() != "}")
				parse_ram_item();
			get_token();
		} else {
			parse_ram_item();
		}
	}

	void parse_top_block() {
		if (peek_token() == "{") {
			get_token();
			while (peek_token() != "}")
				parse_top_item();
			get_token();
		} else {
			parse_top_item();
		}
	}

	void parse_port_item() {
		std::string token = get_token();
		if (token == "ifdef") {
			bool save = enter_ifdef(true);
			parse_port_block();
			if (peek_token() == "else") {
				enter_else(save);
				parse_port_block();
			}
			active = save;
		} else if (token == "ifndef") {
			bool save = enter_ifdef(false);
			parse_port_block();
			if (peek_token() == "else") {
				enter_else(save);
				parse_port_block();
			}
			active = save;
		} else if (token == "option") {
			enter_option();
			parse_port_block();
			exit_option();
		} else if (token == "portoption") {
			enter_portoption();
			parse_port_block();
			exit_portoption();
		} else if (token == "clock") {
			if (port.kind == PortKind::Ar) {
				log_error("%s:%d: `clock` not allowed in async read port.\n", filename.c_str(), line_number);
			}
			ClockDef def;
			token = peek_token();
			if (token == "anyedge") {
				def.kind = ClkPolKind::Anyedge;
				get_token();
			} else if (token == "posedge") {
				def.kind = ClkPolKind::Posedge;
				get_token();
			} else if (token == "negedge") {
				def.kind = ClkPolKind::Negedge;
				get_token();
			} else {
				log_error("%s:%d: expected `posedge`, `negedge`, or `anyedge`, got `%s`.\n", filename.c_str(), line_number, token.c_str());
			}
			token = peek_token();
			if (peek_string()) {
				def.name = get_string();
			}
			get_semi();
			add_cap(port.clock, def);
		} else if (token == "width") {
			WidthDef def;
			token = peek_token();
			bool is_rw = port.kind == PortKind::Srsw || port.kind == PortKind::Arsw;
			if (token == "tied") {
				get_token();
				if (!is_rw)
					log_error("%s:%d: `tied` only makes sense for read+write ports.\n", filename.c_str(), line_number);
				while (peek_int())
					def.wr_widths.push_back(get_int());
				def.tied = true;
			} else if (token == "mix") {
				get_token();
				if (!is_rw)
					log_error("%s:%d: `mix` only makes sense for read+write ports.\n", filename.c_str(), line_number);
				while (peek_int())
					def.wr_widths.push_back(get_int());
				def.rd_widths = def.wr_widths;
				def.tied = false;
			} else if (token == "rd") {
				get_token();
				if (!is_rw)
					log_error("%s:%d: `rd` only makes sense for read+write ports.\n", filename.c_str(), line_number);
				do {
					def.rd_widths.push_back(get_int());
				} while (peek_int());
				eat_token("wr");
				do {
					def.wr_widths.push_back(get_int());
				} while (peek_int());
				def.tied = false;
			} else if (token == "wr") {
				get_token();
				if (!is_rw)
					log_error("%s:%d: `wr` only makes sense for read+write ports.\n", filename.c_str(), line_number);
				do {
					def.wr_widths.push_back(get_int());
				} while (peek_int());
				eat_token("rd");
				do {
					def.rd_widths.push_back(get_int());
				} while (peek_int());
				def.tied = false;
			} else {
				do {
					def.wr_widths.push_back(get_int());
				} while (peek_int());
				def.tied = true;
			}
			get_semi();
			add_cap(port.width, def);
		} else if (token == "addrce") {
			get_semi();
			add_cap(port.addrce, {});
		} else if (token == "rden") {
			if (port.kind != PortKind::Sr && port.kind != PortKind::Srsw)
				log_error("%s:%d: `rden` only allowed on sync read ports.\n", filename.c_str(), line_number);
			token = get_token();
			RdEnKind val;
			if (token == "none") {
				val = RdEnKind::None;
			} else if (token == "any") {
				val = RdEnKind::Any;
			} else if (token == "write-implies") {
				if (port.kind != PortKind::Srsw)
					log_error("%s:%d: `write-implies` only makes sense for read+write ports.\n", filename.c_str(), line_number);
				val = RdEnKind::WriteImplies;
			} else if (token == "write-excludes") {
				if (port.kind != PortKind::Srsw)
					log_error("%s:%d: `write-excludes` only makes sense for read+write ports.\n", filename.c_str(), line_number);
				val = RdEnKind::WriteExcludes;
			} else {
				log_error("%s:%d: expected `none`, `any`, `write-implies`, or `write-excludes`, got `%s`.\n", filename.c_str(), line_number, token.c_str());
			}
			get_semi();
			add_cap(port.rden, val);
		} else if (token == "rdinitval" || token == "rdsrstval" || token == "rdarstval") {
			if (port.kind != PortKind::Sr && port.kind != PortKind::Srsw)
				log_error("%s:%d: `%s` only allowed on sync read ports.\n", filename.c_str(), line_number, token.c_str());
			ResetValDef def;
			if (token == "rdinitval")
				def.kind = ResetKind::Init;
			else if (token == "rdsrstval")
				def.kind = ResetKind::Sync;
			else if (token == "rdarstval")
				def.kind = ResetKind::Async;
			else
				abort();
			token = peek_token();
			if (token == "none") {
				def.val_kind = ResetValKind::None;
				get_token();
			} else if (token == "zero") {
				def.val_kind = ResetValKind::Zero;
				get_token();
			} else {
				def.val_kind = ResetValKind::Named;
				def.name = get_string();
			}
			get_semi();
			add_cap(port.rdrstval, def);
		} else if (token == "rdsrstmode") {
			if (port.kind != PortKind::Sr && port.kind != PortKind::Srsw)
				log_error("%s:%d: `rdsrstmode` only allowed on sync read ports.\n", filename.c_str(), line_number);
			SrstKind val;
			token = get_token();
			if (token == "en-over-srst") {
				val = SrstKind::EnOverSrst;
			} else if (token == "srst-over-en") {
				val = SrstKind::SrstOverEn;
			} else if (token == "any") {
				val = SrstKind::Any;
			} else {
				log_error("%s:%d: expected `en-over-srst`, `srst-over-en`, or `any`, got `%s`.\n", filename.c_str(), line_number, token.c_str());
			}
			get_semi();
			add_cap(port.rdsrstmode, val);
		} else if (token == "wrprio") {
			if (port.kind == PortKind::Ar || port.kind == PortKind::Sr)
				log_error("%s:%d: `wrprio` only allowed on write ports.\n", filename.c_str(), line_number);
			do {
				add_cap(port.wrprio, get_string());
			} while (peek_string());
			get_semi();
		} else if (token == "wrtrans") {
			if (port.kind == PortKind::Ar || port.kind == PortKind::Sr)
				log_error("%s:%d: `wrtrans` only allowed on write ports.\n", filename.c_str(), line_number);
			token = peek_token();
			WrTransDef def;
			if (token == "self") {
				if (port.kind != PortKind::Srsw)
					log_error("%s:%d: `wrtrans self` only allowed on sync read + sync write ports.\n", filename.c_str(), line_number);
				def.target_kind = TransTargetKind::Self;
				get_token();
			} else if (token == "other") {
				def.target_kind = TransTargetKind::Other;
				get_token();
			} else {
				def.target_kind = TransTargetKind::Named;
				def.target_name = get_string();
			}
			token = get_token();
			if (token == "new") {
				def.kind = TransKind::New;
			} else if (token == "old") {
				def.kind = TransKind::Old;
			} else {
				log_error("%s:%d: expected `new` or `old`, got `%s`.\n", filename.c_str(), line_number, token.c_str());
			}
			get_semi();
			add_cap(port.wrtrans, def);
		} else if (token == "wrcs") {
			if (port.kind == PortKind::Ar || port.kind == PortKind::Sr)
				log_error("%s:%d: `wrcs` only allowed on write ports.\n", filename.c_str(), line_number);
			add_cap(port.wrcs, get_int());
			get_semi();
		} else if (token == "") {
			log_error("%s:%d: unexpected EOF while parsing port item.\n", filename.c_str(), line_number);
		} else {
			log_error("%s:%d: unknown port-level item `%s`.\n", filename.c_str(), line_number, token.c_str());
		}
	}

	void parse_ram_item() {
		std::string token = get_token();
		if (token == "ifdef") {
			bool save = enter_ifdef(true);
			parse_ram_block();
			if (peek_token() == "else") {
				enter_else(save);
				parse_ram_block();
			}
			active = save;
		} else if (token == "ifndef") {
			bool save = enter_ifdef(false);
			parse_ram_block();
			if (peek_token() == "else") {
				enter_else(save);
				parse_ram_block();
			}
			active = save;
		} else if (token == "option") {
			enter_option();
			parse_ram_block();
			exit_option();
		} else if (token == "prune") {
			eat_token("rom");
			get_semi();
			ram.prune_rom = true;
		} else if (token == "abits") {
			MemoryDimsDef dims;
			dims.abits = get_int();
			dims.tied = false;
			eat_token("dbits");
			int last = 0;
			do {
				int w = get_int();
				if (w <= 0)
					log_error("%s:%d: dbits %d not positive.\n", filename.c_str(), line_number, w);
				if (w < last * 2)
					log_error("%s:%d: dbits %d smaller than %d required for progression.\n", filename.c_str(), line_number, w, last * 2);
				last = w;
				dims.dbits.push_back(w);
			} while(peek_int());
			if (GetSize(dims.dbits) - 1 > dims.abits)
				log_error("%s:%d: abits %d too small for dbits progression.\n", filename.c_str(), line_number, dims.abits);
			if (peek_token() == "tied") {
				get_token();
				dims.tied = true;
			}
			if (peek_token() == "resource") {
				get_token();
				dims.resource_name = get_string();
				if (peek_int())
					dims.resource_count = get_int();
				else
					dims.resource_count = 1;
			} else {
				dims.resource_count = 1;
			}
			eat_token("cost");
			dims.cost = get_double();
			get_semi();
			add_cap(ram.dims, dims);
		} else if (token == "byte") {
			int val = get_int();
			if (val <= 0)
				log_error("%s:%d: dbits %d not positive.\n", filename.c_str(), line_number, val);
			add_cap(ram.byte, val);
			get_semi();
		} else if (token == "init") {
			MemoryInitKind kind;
			token = get_token();
			if (token == "zero") {
				kind = MemoryInitKind::Zero;
			} else if (token == "any") {
				kind = MemoryInitKind::Any;
			} else if (token == "none") {
				kind = MemoryInitKind::None;
			} else {
				log_error("%s:%d: expected `zero`, `any`, or `none`, got `%s`.\n", filename.c_str(), line_number, token.c_str());
			}
			get_semi();
			add_cap(ram.init, kind);
		} else if (token == "style") {
			do {
				add_cap(ram.style, get_string());
			} while (peek_string());
			get_semi();
		} else if (token == "port") {
			int orig_line = line_number;
			port = PortGroupDef();
			token = get_token();
			if (token == "ar") {
				port.kind = PortKind::Ar;
			} else if (token == "sr") {
				port.kind = PortKind::Sr;
			} else if (token == "sw") {
				port.kind = PortKind::Sw;
			} else if (token == "arsw") {
				port.kind = PortKind::Arsw;
			} else if (token == "srsw") {
				port.kind = PortKind::Srsw;
			} else {
				log_error("%s:%d: expected `ar`, `sr`, `sw`, `arsw`, or `srsw`, got `%s`.\n", filename.c_str(), line_number, token.c_str());
			}
			do {
				port.names.push_back(get_string());
			} while (peek_string());
			parse_port_block();
			if (active) {
				// Add defaults for some options.
				if (port.kind != PortKind::Ar) {
					if (port.clock.empty()) {
						ClockDef def;
						def.kind = ClkPolKind::Anyedge;
						add_cap(port.clock, def);
					}
				}
				if (port.width.empty()) {
					WidthDef def;
					def.tied = true;
					add_cap(port.width, def);
				}
				// Refuse to guess this one — there is no "safe" default.
				if (port.kind == PortKind::Sr || port.kind == PortKind::Srsw) {
					if (port.rden.empty()) {
						log_error("%s:%d: `rden` capability should be specified.\n", filename.c_str(), orig_line);
					}
				}
				add_cap(ram.ports, port);
			}
		} else if (token == "") {
			log_error("%s:%d: unexpected EOF while parsing ram item.\n", filename.c_str(), line_number);
		} else {
			log_error("%s:%d: unknown ram-level item `%s`.\n", filename.c_str(), line_number, token.c_str());
		}
	}

	void parse_top_item() {
		std::string token = get_token();
		if (token == "ifdef") {
			bool save = enter_ifdef(true);
			parse_top_block();
			if (peek_token() == "else") {
				enter_else(save);
				parse_top_block();
			}
			active = save;
		} else if (token == "ifndef") {
			bool save = enter_ifdef(false);
			parse_top_block();
			if (peek_token() == "else") {
				enter_else(save);
				parse_top_block();
			}
			active = save;
		} else if (token == "ram") {
			int orig_line = line_number;
			ram = RamDef();
			ram.prune_rom = false;
			token = get_token();
			if (token == "distributed") {
				ram.kind = RamKind::Distributed;
			} else if (token == "block") {
				ram.kind = RamKind::Block;
			} else if (token == "huge") {
				ram.kind = RamKind::Huge;
			} else {
				log_error("%s:%d: expected `distributed`, `block`, or `huge`, got `%s`.\n", filename.c_str(), line_number, token.c_str());
			}
			ram.id = get_id();
			parse_ram_block();
			if (active) {
				if (ram.dims.empty())
					log_error("%s:%d: `dims` capability should be specified.\n", filename.c_str(), orig_line);
				if (ram.ports.empty())
					log_error("%s:%d: at least one port group should be specified.\n", filename.c_str(), orig_line);
				pool<std::string> pnedge_clock;
				pool<std::string> anyedge_clock;
				for (auto &port: ram.ports) {
					for (auto &def: port.val.clock) {
						if (def.val.name.empty())
							continue;
						if (def.val.kind == ClkPolKind::Anyedge)
							anyedge_clock.insert(def.val.name);
						else
							pnedge_clock.insert(def.val.name);
					}
				}
				for (auto &x: pnedge_clock)
					if (anyedge_clock.count(x))
						log_error("%s:%d: named clock \"%s\" used with both posedge/negedge and anyedge clocks.\n", filename.c_str(), orig_line, x.c_str());
				validate_widths(ram);
				lib.ram_defs.push_back(ram);
			}
		} else if (token == "") {
			log_error("%s:%d: unexpected EOF while parsing top item.\n", filename.c_str(), line_number);
		} else {
			log_error("%s:%d: unknown top-level item `%s`.\n", filename.c_str(), line_number, token.c_str());
		}
	}

	void validate_widths(const RamDef &ram) {
		for (auto &ddef: ram.dims) {
			auto &dbits = ddef.val.dbits;
			for (auto &bdef: ram.byte) {
				if (opts_conflict(ddef.opts, bdef.opts))
					continue;
				int byte = bdef.val;
				bool ok = false;
				if (dbits[0] % byte == 0)
					ok = true;
				if (byte % dbits.back() == 0)
					ok = true;
				for (auto x: dbits)
					if (x == byte)
						ok = true;
				if (!ok) {
					log_error("%s:%d: byte width %d invalid for dbits.\n", filename.c_str(), line_number, byte);
				}
			}
			for (auto &pdef: ram.ports) {
				if (opts_conflict(ddef.opts, pdef.opts))
					continue;
				for (auto &wdef: pdef.val.width) {
					if (opts_conflict(ddef.opts, wdef.opts))
						continue;
					if (ddef.val.tied && !wdef.val.wr_widths.empty()) {
						log_error("%s:%d: per-port width doesn't make sense for tied dbits.\n", filename.c_str(), line_number);
					}
					validate_widthdef(dbits, wdef.val.wr_widths);
					validate_widthdef(dbits, wdef.val.rd_widths);
				}
			}
		}
	}

	void validate_widthdef(const std::vector<int> &dbits, const std::vector<int> widths) {
		if (widths.empty())
			return;
		for (int i = 0; i < GetSize(dbits); i++) {
			if (dbits[i] == widths[0]) {
				for (int j = 0; j < GetSize(widths); j++) {
					if (i+j >= GetSize(dbits) || dbits[i+j] != widths[j]) {
						log_error("%s:%d: port width %d doesn't match dbits progression.\n", filename.c_str(), line_number, widths[j]);
					}
				}
				return;
			}
		}
		log_error("%s:%d: port width %d invalid for dbits.\n", filename.c_str(), line_number, widths[0]);
	}

	void parse() {
		while (peek_token() != "")
			parse_top_item();
	}
};

struct WrPortConfig {
	// Index of the read port this port is merged with, or -1 if none.
	int rd_port;
	// Index of the PortGroupDef in the RamDef.
	int port_def;
	// Already-decided port option settings.
	Options portopts;
	// Emulate priority logic for this list of (source) write port indices.
	std::vector<int> emu_prio;
	ClkPolKind clkpol_kind;
	// The chosen widths def.
	int width_def;

	WrPortConfig() : rd_port(-1) {}
};

struct RdPortConfig {
	// Index of the write port this port is merged with, or -1 if none.
	int wr_port;
	// Index of the PortGroupDef in the RamDef.
	int port_def;
	// Already-decided port option settings.  Unused if wr_port is not -1:
	// in this case, use write port's portopts instead.
	Options portopts;
	// The named reset value assignments.
	dict<std::string, Const> resetvals;
	// If true, this is a sync port mapped into async mem, make an output
	// register.  Exclusive with the following options.
	bool emu_sync;
	// Emulate the EN / ARST / SRST / init value circuitry.
	bool emu_en;
	bool emu_arst;
	bool emu_srst;
	bool emu_init;
	// Emulate EN-SRST priority.
	bool emu_srst_en_prio;
	bool emit_en;
	// Emulate transparency logic for this list of (source) write port indices.
	std::vector<int> emu_trans;
	ClkPolKind clkpol_kind;
	// The chosen widths def.
	int width_def;

	RdPortConfig() : wr_port(-1), emu_sync(false), emu_en(false), emu_arst(false), emu_srst(false), emu_init(false), emu_srst_en_prio(false), emit_en(false) {}
};

struct MemConfig {
	// Index of the RamDef in the Library.
	int ram_def;
	// Already-decided option settings.
	Options opts;
	// Port assignments, indexed by Mem port index.
	std::vector<WrPortConfig> wr_ports;
	std::vector<RdPortConfig> rd_ports;
	// The named clock and clock polarity assignments.
	// For anyedge clocks: the bool is the shared clock polarity.
	// For pos/negedge clocks: the bool is the "needs inversion" flag.
	dict<std::string, std::pair<SigBit, bool>> clocks_anyedge;
	dict<std::string, std::pair<SigBit, bool>> clocks_pnedge;
	// Emulate read-first write-read behavior using soft logic.
	bool emu_read_first;
	// The chosen dims def.
	int dims_def;
	// Chosen byte width.
	int byte;
	// This many low bits of (target) address are always-0 on all ports.
	int base_width_log2;
	int unit_width_log2;
	std::vector<int> swizzle;
	int hard_wide_mask;
	int emu_wide_mask;
	// How many times the base memory block will need to be duplicated to get more
	// data bits.
	int repl_d;
	// How many times the whole memory array will need to be duplicated to cover
	// all read ports required.
	int repl_port;
	// Emulation score — how much circuitry we need to add for priority / transparency /
	// reset / initial value emulation.
	int score_emu;
	// Mux score — how much circuitry we need to add to manually decode whatever address
	// bits are not decoded by the memory array itself, for reads.
	int score_mux;
	// Demux score — how much circuitry we need to add to manually decode whatever address
	// bits are not decoded by the memory array itself, for writes.
	int score_demux;
	double cost;
};

typedef std::vector<MemConfig> MemConfigs;

bool opts_applied(const Options &dst, const Options &src) {
	for (auto &it: src) {
		auto it2 = dst.find(it.first);
		if (it2 == dst.end())
			return false;
		if (it2->second != it.second)
			return false;
	}
	return true;
}

bool apply_opts(Options &dst, const Options &src) {
	for (auto &it: src) {
		auto it2 = dst.find(it.first);
		if (it2 == dst.end())
			dst[it.first] = it.second;
		else if (it2->second != it.second)
			return false;
	}
	return true;
}

template<typename T>
bool apply_wrport_opts(MemConfig &cfg, int pidx, const Capability<T> &cap) {
	auto &pcfg = cfg.wr_ports[pidx];
	return apply_opts(cfg.opts, cap.opts) && apply_opts(pcfg.portopts, cap.portopts);
}

template<typename T>
bool apply_rdport_opts(MemConfig &cfg, int pidx, const Capability<T> &cap) {
	auto &pcfg = cfg.rd_ports[pidx];
	if (pcfg.wr_port != -1)
		return apply_wrport_opts(cfg, pcfg.wr_port, cap);
	return apply_opts(cfg.opts, cap.opts) && apply_opts(pcfg.portopts, cap.portopts);
}

template<typename T>
bool wrport_opts_applied(const MemConfig &cfg, int pidx, const Capability<T> &cap) {
	auto &pcfg = cfg.wr_ports[pidx];
	return opts_applied(cfg.opts, cap.opts) && opts_applied(pcfg.portopts, cap.portopts);
}

template<typename T>
bool rdport_opts_applied(MemConfig &cfg, int pidx, const Capability<T> &cap) {
	auto &pcfg = cfg.rd_ports[pidx];
	if (pcfg.wr_port != -1)
		return wrport_opts_applied(cfg, pcfg.wr_port, cap);
	return opts_applied(cfg.opts, cap.opts) && opts_applied(pcfg.portopts, cap.portopts);
}

bool apply_clock(MemConfig &cfg, const ClockDef &def, SigBit clk, bool clk_polarity) {
	if (def.name.empty())
		return true;
	if (def.kind == ClkPolKind::Anyedge) {
		auto it = cfg.clocks_anyedge.find(def.name);
		if (it == cfg.clocks_anyedge.end()) {
			cfg.clocks_anyedge.insert({def.name, {clk, clk_polarity}});
			return true;
		} else {
			return it->second == std::make_pair(clk, clk_polarity);
		}
	} else {
		bool flip = clk_polarity ^ (def.kind == ClkPolKind::Posedge);
		auto it = cfg.clocks_pnedge.find(def.name);
		if (it == cfg.clocks_pnedge.end()) {
			cfg.clocks_pnedge.insert({def.name, {clk, flip}});
			return true;
		} else {
			return it->second == std::make_pair(clk, flip);
		}
	}
}

bool apply_rstval(RdPortConfig &pcfg, const ResetValDef &def, Const val) {
	if (def.val_kind == ResetValKind::None)
		return false;
	if (def.val_kind == ResetValKind::Zero) {
		for (auto bit: val.bits)
			if (bit == State::S1)
				return false;
		return true;
	} else {
		auto it = pcfg.resetvals.find(def.name);
		if (it == pcfg.resetvals.end()) {
			pcfg.resetvals.insert({def.name, val});
			return true;
		} else {
			return it->second == val;
		}
	}
}

struct MapWorker {
	Module *module;
	ModWalker modwalker;
	SigMap sigmap;
	SigMap sigmap_xmux;
	FfInitVals initvals;

	MapWorker(Module *module) : module(module), modwalker(module->design, module), sigmap(module), sigmap_xmux(module), initvals(&sigmap, module) {
		for (auto cell : module->cells())
		{
			if (cell->type == ID($mux))
			{
				RTLIL::SigSpec sig_a = sigmap_xmux(cell->getPort(ID::A));
				RTLIL::SigSpec sig_b = sigmap_xmux(cell->getPort(ID::B));

				if (sig_a.is_fully_undef())
					sigmap_xmux.add(cell->getPort(ID::Y), sig_b);
				else if (sig_b.is_fully_undef())
					sigmap_xmux.add(cell->getPort(ID::Y), sig_a);
			}
		}
	}
};

struct SwizzleBit {
	bool valid;
	int mux_idx;
	int addr;
	int bit;
};

struct Swizzle {
	int addr_shift;
	std::vector<int> addr_mux_bits;
	std::vector<std::vector<SwizzleBit>> bits;
};

struct MemMapping {
	MapWorker &worker;
	QuickConeSat qcsat;
	Mem &mem;
	const Library &lib;
	std::vector<MemConfig> cfgs;
	bool logic_ok;
	double logic_cost;
	RamKind kind;
	std::string style;
	dict<int, int> wr_en_cache;
	dict<std::pair<int, int>, bool> wr_implies_rd_cache;
	dict<std::pair<int, int>, bool> wr_excludes_rd_cache;

	MemMapping(MapWorker &worker, Mem &mem, Library &lib) : worker(worker), qcsat(worker.modwalker), mem(mem), lib(lib) {
		determine_style();
		logic_ok = determine_logic_ok();
		logic_cost = mem.width * mem.size;
		if (kind == RamKind::Logic)
			return;
		for (int i = 0; i < GetSize(lib.ram_defs); i++) {
			MemConfig cfg;
			cfg.ram_def = i;
			cfg.emu_read_first = false;
			cfgs.push_back(cfg);
		}
		handle_ram_kind();
		handle_ram_style();
		handle_init();
		handle_wr_ports();
		handle_rd_ports();
		handle_trans();
		// If we got this far, the memory is mappable.  The following two can require emulating
		// some functionality, but cannot cause the mapping to fail.
		handle_priority();
		handle_rd_init();
		handle_rd_arst();
		handle_rd_srst();
		score_emu_ports();
		// Now it is just a matter of picking geometry.
		dump_configs(0);
		handle_geom_split();
		dump_configs(1);
		prune_pre_geom();
		dump_configs(2);
		handle_geom();
		dump_configs(3);
		prune_post_geom();
		dump_configs(4);
	}

	bool addr_compatible(int wpidx, int rpidx) {
		auto &wport = mem.wr_ports[wpidx];
		auto &rport = mem.rd_ports[rpidx];
		int max_wide_log2 = std::max(rport.wide_log2, wport.wide_log2);
		SigSpec raddr = rport.addr.extract_end(max_wide_log2);
		SigSpec waddr = wport.addr.extract_end(max_wide_log2);
		int abits = std::max(GetSize(raddr), GetSize(waddr));
		raddr.extend_u0(abits);
		waddr.extend_u0(abits);
		return worker.sigmap_xmux(raddr) == worker.sigmap_xmux(waddr);
	}

	int get_wr_en(int wpidx) {
		auto it = wr_en_cache.find(wpidx);
		if (it != wr_en_cache.end())
			return it->second;
		int res = qcsat.ez->expression(qcsat.ez->OpOr, qcsat.importSig(mem.wr_ports[wpidx].en));
		wr_en_cache.insert({wpidx, res});
		return res;
	}

	bool get_wr_implies_rd(int wpidx, int rpidx) {
		auto key = std::make_pair(wpidx, rpidx);
		auto it = wr_implies_rd_cache.find(key);
		if (it != wr_implies_rd_cache.end())
			return it->second;
		int wr_en = get_wr_en(wpidx);
		int rd_en = qcsat.importSigBit(mem.rd_ports[rpidx].en[0]);
		qcsat.prepare();
		bool res = !qcsat.ez->solve(wr_en, qcsat.ez->NOT(rd_en));
		wr_implies_rd_cache.insert({key, res});
		return res;
	}

	bool get_wr_excludes_rd(int wpidx, int rpidx) {
		auto key = std::make_pair(wpidx, rpidx);
		auto it = wr_excludes_rd_cache.find(key);
		if (it != wr_excludes_rd_cache.end())
			return it->second;
		int wr_en = get_wr_en(wpidx);
		int rd_en = qcsat.importSigBit(mem.rd_ports[rpidx].en[0]);
		qcsat.prepare();
		bool res = !qcsat.ez->solve(wr_en, rd_en);
		wr_excludes_rd_cache.insert({key, res});
		return res;
	}

	void dump_configs(int stage);
	void dump_config(MemConfig &cfg, int stage);
	void determine_style();
	bool determine_logic_ok();
	void handle_ram_kind();
	void handle_ram_style();
	void handle_init();
	void handle_wr_ports();
	void handle_rd_ports();
	void handle_trans();
	void handle_priority();
	void handle_rd_init();
	void handle_rd_arst();
	void handle_rd_srst();
	void score_emu_ports();
	void handle_geom_split();
	void prune_pre_geom();
	void handle_geom();
	void prune_post_geom();
	void emit(const MemConfig &cfg);
	Swizzle gen_swizzle(const MemConfig &cfg, int sw_wide_log2, int hw_wide_log2);
};

void MemMapping::dump_configs(int stage) {
	const char *stage_name;
	switch (stage) {
		case 0:
			stage_name = "after initial split";
			break;
		case 1:
			stage_name = "after geometry split";
			break;
		case 2:
			stage_name = "after pre-geometry prune";
			break;
		case 3:
			stage_name = "post-geometry";
			break;
		case 4:
			stage_name = "after post-geometry prune";
			break;
		default:
			abort();
	}
	log_debug("Memory %s.%s mapping candidates (%s):\n", log_id(mem.module->name), log_id(mem.memid), stage_name);
	if (logic_ok)
		log_debug("- logic fallback\n");
	for (auto &cfg: cfgs) {
		dump_config(cfg, stage);
	}
}

void MemMapping::dump_config(MemConfig &cfg, int stage) {
	auto &rdef = lib.ram_defs[cfg.ram_def];
	log_debug("- %s:\n", log_id(rdef.id));
	for (auto &it: cfg.opts)
		log_debug("  - option %s %s\n", it.first.c_str(), log_const(it.second));
	log_debug("  - emulation score: %d\n", cfg.score_emu);
	log_debug("  - replicates (for ports): %d\n", cfg.repl_port);
	if (stage >= 3) {
		log_debug("  - replicates (for data): %d\n", cfg.repl_d);
		log_debug("  - mux score: %d\n", cfg.score_mux);
		log_debug("  - demux score: %d\n", cfg.score_demux);
		log_debug("  - cost: %f\n", cfg.cost);
	}
	if (stage >= 1) {
		auto &dims = rdef.dims[cfg.dims_def].val;
		std::stringstream os;
		for (int x: dims.dbits)
			os << " " << x;
		std::string dbits_s = os.str();
		log_debug("  - abits %d dbits%s\n", dims.abits, dbits_s.c_str());
		if (cfg.byte != 0)
			log_debug("  - byte width %d\n", cfg.byte);
		if (stage >= 3) {
			log_debug("  - chosen base width %d\n", dims.dbits[cfg.base_width_log2]);
			os.str("");
			for (int x: cfg.swizzle)
				if (x == -1)
					os << " -";
				else
					os << " " << x;
			std::string swizzle_s = os.str();
			log_debug("  - swizzle%s\n", swizzle_s.c_str());
			os.str("");
			for (int i = 0; (1 << i) <= cfg.hard_wide_mask; i++)
				if (cfg.hard_wide_mask & 1 << i)
					os << " " << i;
			std::string wide_s = os.str();
			if (cfg.hard_wide_mask)
				log_debug("  - hard wide bits%s\n", wide_s.c_str());
		}
	}
	if (cfg.emu_read_first)
		log_debug("  - emulate read-first behavior\n");
	for (int i = 0; i < GetSize(mem.wr_ports); i++) {
		auto &pcfg = cfg.wr_ports[i];
		auto &pdef = rdef.ports[pcfg.port_def].val;
		if (pcfg.rd_port == -1)
			log_debug("  - write port %d: port group %s\n", i, pdef.names[0].c_str());
		else
			log_debug("  - write port %d: port group %s (shared with read port %d)\n", i, pdef.names[0].c_str(), pcfg.rd_port);

		for (auto &it: pcfg.portopts)
			log_debug("    - option %s %s\n", it.first.c_str(), log_const(it.second));
		if (stage >= 1) {
			auto &wdef = pdef.width[pcfg.width_def].val;
			std::stringstream os;
			for (int x: wdef.wr_widths)
				os << " " << x;
			std::string widths_s = os.str();
			if (widths_s.empty()) {
				auto &dims = rdef.dims[cfg.dims_def].val;
				if (pcfg.rd_port != -1 && GetSize(dims.dbits) > 1) {
					log_debug("    - width %s\n", wdef.tied ? "tied" : "independent");
				}
			} else {
				const char *note = "";
				if (pcfg.rd_port != -1)
					note = wdef.tied ? " (tied)" : " (independent)";
				log_debug("    - widths%s%s\n", widths_s.c_str(), note);
			}
		}
		for (auto i: pcfg.emu_prio)
			log_debug("    - emulate priority over write port %d\n", i);
	}
	for (int i = 0; i < GetSize(mem.rd_ports); i++) {
		auto &pcfg = cfg.rd_ports[i];
		auto &pdef = rdef.ports[pcfg.port_def].val;
		if (pcfg.wr_port == -1)
			log_debug("  - read port %d: port group %s\n", i, pdef.names[0].c_str());
		else
			log_debug("  - read port %d: port group %s (shared with write port %d)\n", i, pdef.names[0].c_str(), pcfg.wr_port);
		for (auto &it: pcfg.portopts)
			log_debug("    - option %s %s\n", it.first.c_str(), log_const(it.second));
		if (stage >= 1) {
			auto &wdef = pdef.width[pcfg.width_def].val;
			std::stringstream os;
			for (int x: wdef.tied ? wdef.wr_widths : wdef.rd_widths)
				os << " " << x;
			std::string widths_s = os.str();
			if (widths_s.empty()) {
				auto &dims = rdef.dims[cfg.dims_def].val;
				if (pcfg.wr_port != -1 && GetSize(dims.dbits) > 1) {
					log_debug("    - width %s\n", wdef.tied ? "tied" : "independent");
				}
			} else {
				const char *note = "";
				if (pcfg.wr_port != -1)
					note = wdef.tied ? " (tied)" : " (independent)";
				log_debug("    - widths%s%s\n", widths_s.c_str(), note);
			}
		}
		if (pcfg.emu_sync)
			log_debug("    - emulate data register\n");
		if (pcfg.emu_en)
			log_debug("    - emulate clock enable\n");
		if (pcfg.emu_arst)
			log_debug("    - emulate async reset\n");
		if (pcfg.emu_srst)
			log_debug("    - emulate sync reset\n");
		if (pcfg.emu_init)
			log_debug("    - emulate init value\n");
		if (pcfg.emu_srst_en_prio)
			log_debug("    - emulate sync reset / enable priority\n");
		for (auto i: pcfg.emu_trans)
			log_debug("    - emulate transparency with write port %d\n", i);
	}
}

// Go through memory attributes to determine user-requested mapping style.
void MemMapping::determine_style() {
	kind = RamKind::Auto;
	style = "";
	for (auto attr: {ID::ram_block, ID::rom_block, ID::ram_style, ID::rom_style, ID::ramstyle, ID::romstyle, ID::syn_ramstyle, ID::syn_romstyle}) {
		if (mem.has_attribute(attr)) {
			Const val = mem.attributes.at(attr);
			if (val == 1) {
				kind = RamKind::NotLogic;
				return;
			}
			std::string val_s = val.decode_string();
			if (val_s == "auto") {
				// Nothing.
			} else if (val_s == "logic" || val_s == "registers") {
				kind = RamKind::Logic;
			} else if (val_s == "distributed") {
				kind = RamKind::Distributed;
			} else if (val_s == "block" || val_s == "block_ram" || val_s == "ebr") {
				kind = RamKind::Block;
			} else if (val_s == "huge" || val_s == "ultra") {
				kind = RamKind::Huge;
			} else {
				kind = RamKind::NotLogic;
				style = val_s;
			}
			return;
		}
	}
	if (mem.get_bool_attribute(ID::logic_block))
		kind = RamKind::Logic;
}

// Determine whether the memory can be mapped entirely to soft logic.
bool MemMapping::determine_logic_ok() {
	if (kind != RamKind::Auto && kind != RamKind::Logic)
		return false;
	// Memory is mappable entirely to soft logic iff all its write ports are in the same clock domain.
	if (mem.wr_ports.empty())
		return true;
	for (auto &port: mem.wr_ports) {
		if (!port.clk_enable)
			return false;
		if (port.clk != mem.wr_ports[0].clk)
			return false;
		if (port.clk_polarity != mem.wr_ports[0].clk_polarity)
			return false;
	}
	return true;
}

// Apply RAM kind restrictions (logic/distributed/block/huge), if any.
void MemMapping::handle_ram_kind() {
	if (style != "")
		return;
	MemConfigs new_cfgs;
	for (auto &cfg: cfgs) {
		auto &rdef = lib.ram_defs[cfg.ram_def];
		bool pass = false;
		if (rdef.kind == kind)
			pass = true;
		if (kind == RamKind::Auto || kind == RamKind::NotLogic) {
			pass = true;
			if (kind == RamKind::Distributed && lib.opts.no_auto_distributed)
				pass = false;
			if (kind == RamKind::Block && lib.opts.no_auto_block)
				pass = false;
			if (kind == RamKind::Huge && lib.opts.no_auto_huge)
				pass = false;
		}
		if (pass)
			new_cfgs.push_back(cfg);
	}
	cfgs = new_cfgs;
	if (cfgs.empty()) {
		const char *kind_s = "";
		switch (kind) {
			case RamKind::Distributed:
				kind_s = "distributed ";
				break;
			case RamKind::Block:
				kind_s = "block ";
				break;
			case RamKind::Huge:
				kind_s = "huge ";
				break;
			case RamKind::NotLogic:
				kind_s = "";
				break;
			default:
				return;
		}
		log_error("%s.%s: no available %sRAMs\n", log_id(mem.module->name), log_id(mem.memid), kind_s);
	}
}

// Apply specific RAM style restrictions, if any.
void MemMapping::handle_ram_style() {
	if (style == "")
		return;
	MemConfigs new_cfgs;
	for (auto &cfg: cfgs) {
		for (auto &def: lib.ram_defs[cfg.ram_def].style) {
			if (def.val != style)
				continue;
			MemConfig new_cfg = cfg;
			if (!apply_opts(new_cfg.opts, def.opts))
				continue;
			new_cfgs.push_back(new_cfg);
		}
	}
	cfgs = new_cfgs;
	if (cfgs.empty())
		log_error("%s.%s: no available RAMs with style \"%s\"\n", log_id(mem.module->name), log_id(mem.memid), style.c_str());
}

// Handle memory initializer restrictions, if any.
void MemMapping::handle_init() {
	bool has_nonx = false;
	bool has_one = false;

	for (auto &init: mem.inits) {
		if (init.data.is_fully_undef())
			continue;
		has_nonx = true;
		for (auto bit: init.data)
			if (bit == State::S1)
				has_one = true;
	}

	if (!has_nonx)
		return;

	MemConfigs new_cfgs;
	for (auto &cfg: cfgs) {
		for (auto &def: lib.ram_defs[cfg.ram_def].init) {
			if (has_one) {
				if (def.val != MemoryInitKind::Any)
					continue;
			} else {
				if (def.val != MemoryInitKind::Any && def.val != MemoryInitKind::Zero)
					continue;
			}
			MemConfig new_cfg = cfg;
			if (!apply_opts(new_cfg.opts, def.opts))
				continue;
			new_cfgs.push_back(new_cfg);
		}
	}
	cfgs = new_cfgs;
}

// Perform write port assignment, validating clock options as we go.
void MemMapping::handle_wr_ports() {
	if (mem.wr_ports.empty()) {
		MemConfigs new_cfgs;
		for (auto &cfg: cfgs) {
			auto &ram_def = lib.ram_defs[cfg.ram_def];
			if (!ram_def.prune_rom)
				new_cfgs.push_back(cfg);
		}
		cfgs = new_cfgs;
	}
	for (auto &port: mem.wr_ports) {
		if (!port.clk_enable) {
			// Async write ports not supported.
			cfgs.clear();
			return;
		}
		MemConfigs new_cfgs;
		for (auto &cfg: cfgs) {
			auto &ram_def = lib.ram_defs[cfg.ram_def];
			for (int i = 0; i < GetSize(ram_def.ports); i++) {
				auto &def = ram_def.ports[i];
				// Make sure the target is a write port.
				if (def.val.kind == PortKind::Ar || def.val.kind == PortKind::Sr)
					continue;
				// Make sure the target port group still has a free port.
				int used = 0;
				for (auto &oport: cfg.wr_ports)
					if (oport.port_def == i)
						used++;
				if (used >= GetSize(def.val.names))
					continue;
				// Apply the options.
				MemConfig cfg2 = cfg;
				if (!apply_opts(cfg2.opts, def.opts))
					continue;
				WrPortConfig pcfg2;
				pcfg2.rd_port = -1;
				pcfg2.port_def = i;
				// Pick a clock def.
				for (auto &cdef: def.val.clock) {
					MemConfig cfg3 = cfg2;
					WrPortConfig pcfg3 = pcfg2;
					if (!apply_opts(cfg3.opts, cdef.opts))
						continue;
					if (!apply_opts(pcfg3.portopts, cdef.portopts))
						continue;
					if (!apply_clock(cfg3, cdef.val, port.clk, port.clk_polarity))
						continue;
					pcfg3.clkpol_kind = cdef.val.kind;
					cfg3.wr_ports.push_back(pcfg3);
					new_cfgs.push_back(cfg3);
				}
			}
		}
		cfgs = new_cfgs;
	}
}

// Perform read port assignment, validating clock and rden options as we go.
void MemMapping::handle_rd_ports() {
	for (int pidx = 0; pidx < GetSize(mem.rd_ports); pidx++) {
		auto &port = mem.rd_ports[pidx];
		MemConfigs new_cfgs;
		for (auto &cfg: cfgs) {
			auto &ram_def = lib.ram_defs[cfg.ram_def];
			// First pass: read port not shared with a write port.
			for (int i = 0; i < GetSize(ram_def.ports); i++) {
				auto &def = ram_def.ports[i];
				// Make sure the target is a read port.
				if (def.val.kind == PortKind::Sw)
					continue;
				// If mapping an async port, accept only async defs.
				if (!port.clk_enable) {
					if (def.val.kind == PortKind::Sr || def.val.kind == PortKind::Srsw)
						continue;
				}
				// Make sure the target port group has a port not used up by write ports.
				// Overuse by other read ports is not a problem — this will just result
				// in memory duplication.
				int used = 0;
				for (auto &oport: cfg.wr_ports)
					if (oport.port_def == i)
						used++;
				if (used >= GetSize(def.val.names))
					continue;
				// Apply the options.
				MemConfig cfg2 = cfg;
				if (!apply_opts(cfg2.opts, def.opts))
					continue;
				RdPortConfig pcfg2;
				pcfg2.wr_port = -1;
				pcfg2.port_def = i;
				if (def.val.kind == PortKind::Sr || def.val.kind == PortKind::Srsw) {
					pcfg2.emu_sync = false;
					// Pick a clock def.
					for (auto &cdef: def.val.clock) {
						MemConfig cfg3 = cfg2;
						RdPortConfig pcfg3 = pcfg2;
						if (!apply_opts(cfg3.opts, cdef.opts))
							continue;
						if (!apply_opts(pcfg3.portopts, cdef.portopts))
							continue;
						if (!apply_clock(cfg3, cdef.val, port.clk, port.clk_polarity))
							continue;
						pcfg3.clkpol_kind = cdef.val.kind;
						// Pick a rden def.
						for (auto &endef: def.val.rden) {
							MemConfig cfg4 = cfg3;
							RdPortConfig pcfg4 = pcfg3;
							if (!apply_opts(cfg4.opts, endef.opts))
								continue;
							if (!apply_opts(pcfg4.portopts, endef.portopts))
								continue;
							if (endef.val == RdEnKind::None && port.en != State::S1) {
								pcfg4.emu_en = true;
							}
							pcfg4.emit_en = endef.val != RdEnKind::None;
							cfg4.rd_ports.push_back(pcfg4);
							new_cfgs.push_back(cfg4);
						}
					}
				} else {
					pcfg2.emu_sync = port.clk_enable;
					cfg2.rd_ports.push_back(pcfg2);
					new_cfgs.push_back(cfg2);
				}
			}
			// Second pass: read port shared with a write port.
			for (int wpidx = 0; wpidx < GetSize(mem.wr_ports); wpidx++) {
				auto &wport = mem.wr_ports[wpidx];
				int didx = cfg.wr_ports[wpidx].port_def;
				auto &def = ram_def.ports[didx];
				// Make sure the write port is not yet shared.
				if (cfg.wr_ports[wpidx].rd_port != -1)
					continue;
				// Make sure the target is a read port.
				if (def.val.kind == PortKind::Sw)
					continue;
				// Validate address compatibility.
				if (!addr_compatible(wpidx, pidx))
					continue;
				// Validate clock compatibility, if needed.
				if (def.val.kind == PortKind::Srsw) {
					if (!port.clk_enable)
						continue;
					if (port.clk != wport.clk)
						continue;
					if (port.clk_polarity != wport.clk_polarity)
						continue;
				}
				// Okay, let's fill it in.
				MemConfig cfg2 = cfg;
				cfg2.wr_ports[wpidx].rd_port = pidx;
				RdPortConfig pcfg2;
				pcfg2.wr_port = wpidx;
				pcfg2.port_def = didx;
				pcfg2.emu_sync = port.clk_enable && def.val.kind == PortKind::Arsw;
				// For srsw, pick rden capability.
				if (def.val.kind == PortKind::Srsw) {
					for (auto &endef: def.val.rden) {
						MemConfig cfg3 = cfg2;
						RdPortConfig pcfg3 = pcfg2;
						if (!apply_wrport_opts(cfg3, wpidx, endef))
							continue;
						switch (endef.val) {
							case RdEnKind::None:
								pcfg3.emu_en = port.en != State::S1;
								break;
							case RdEnKind::Any:
								break;
							case RdEnKind::WriteImplies:
								pcfg3.emu_en = !get_wr_implies_rd(wpidx, pidx);
								break;
							case RdEnKind::WriteExcludes:
								if (!get_wr_excludes_rd(wpidx, pidx))
									continue;
								break;
						}
						pcfg3.emit_en = endef.val != RdEnKind::None;
						cfg3.rd_ports.push_back(pcfg3);
						new_cfgs.push_back(cfg3);
					}
				} else {
					cfg2.rd_ports.push_back(pcfg2);
					new_cfgs.push_back(cfg2);
				}
			}
		}
		cfgs = new_cfgs;
	}
}

// Validate transparency restrictions, determine where to add soft transparency logic.
void MemMapping::handle_trans() {
	if (mem.emulate_read_first_ok()) {
		MemConfigs new_cfgs;
		for (auto &cfg: cfgs) {
			new_cfgs.push_back(cfg);
			bool ok = true;
			// Using this trick will break read-write port sharing.
			for (auto &pcfg: cfg.rd_ports)
				if (pcfg.wr_port != -1)
					ok = false;
			if (ok) {
				cfg.emu_read_first = true;
				new_cfgs.push_back(cfg);
			}
		}
		cfgs = new_cfgs;
	}
	for (int rpidx = 0; rpidx < GetSize(mem.rd_ports); rpidx++) {
		auto &rport = mem.rd_ports[rpidx];
		if (!rport.clk_enable)
			continue;
		for (int wpidx = 0; wpidx < GetSize(mem.wr_ports); wpidx++) {
			auto &wport = mem.wr_ports[wpidx];
			if (!wport.clk_enable)
				continue;
			if (rport.clk != wport.clk)
				continue;
			if (rport.clk_polarity != wport.clk_polarity)
				continue;
			// If we got this far, we have a transparency restriction
			// to uphold.
			MemConfigs new_cfgs;
			for (auto &cfg: cfgs) {
				auto &rpcfg = cfg.rd_ports[rpidx];
				auto &wpcfg = cfg.wr_ports[wpidx];
				auto &rdef = lib.ram_defs[cfg.ram_def];
				auto &wpdef = rdef.ports[wpcfg.port_def];
				auto &rpdef = rdef.ports[rpcfg.port_def];
				if (rport.collision_x_mask[wpidx] && !cfg.emu_read_first) {
					new_cfgs.push_back(cfg);
					continue;
				}
				bool transparent = rport.transparency_mask[wpidx] || cfg.emu_read_first;
				if (rpcfg.emu_sync) {
					// For async read port, just add the transparency logic
					// if necessary.
					if (transparent)
						rpcfg.emu_trans.push_back(wpidx);
					new_cfgs.push_back(cfg);
				} else {
					// Otherwise, split through the relevant wrtrans caps.
					// For non-transparent ports, the cap needs to be present.
					// For transparent ports, we can emulate transparency
					// even without a direct cap.
					bool found_free = false;
					for (auto &tdef: wpdef.val.wrtrans) {
						// Check if the target matches.
						switch (tdef.val.target_kind) {
							case TransTargetKind::Self:
								if (wpcfg.rd_port != rpidx)
									continue;
								break;
							case TransTargetKind::Other:
								if (wpcfg.rd_port == rpidx)
									continue;
								break;
							case TransTargetKind::Named:
								if (rpdef.val.names[0] != tdef.val.target_name)
									continue;
								break;
						}
						// Check if the transparency kind is acceptable.
						if (transparent) {
							if (tdef.val.kind == TransKind::Old)
								continue;
						} else {
							if (tdef.val.kind != TransKind::Old)
								continue;
						}
						// Okay, we can use this cap.
						MemConfig cfg2 = cfg;
						if (wrport_opts_applied(cfg2, wpidx, tdef))
							found_free = true;
						else if (!apply_wrport_opts(cfg2, wpidx, tdef))
							continue;
						new_cfgs.push_back(cfg2);
					}
					if (!found_free && transparent) {
						// If the port pair is transparent, but no cap was
						// found, or the cap found had a splitting cost
						// to it, consider emulation as well.
						rpcfg.emu_trans.push_back(wpidx);
						new_cfgs.push_back(cfg);
					}
				}
			}
			cfgs = new_cfgs;
		}
	}
}

// Determine where to add soft priority logic.
void MemMapping::handle_priority() {
	for (int p1idx = 0; p1idx < GetSize(mem.wr_ports); p1idx++) {
		for (int p2idx = 0; p2idx < GetSize(mem.wr_ports); p2idx++) {
			auto &port2 = mem.wr_ports[p2idx];
			if (!port2.priority_mask[p1idx])
				continue;
			MemConfigs new_cfgs;
			for (auto &cfg: cfgs) {
				auto &p1cfg = cfg.rd_ports[p1idx];
				auto &p2cfg = cfg.wr_ports[p2idx];
				auto &rdef = lib.ram_defs[cfg.ram_def];
				auto &p1def = rdef.ports[p1cfg.port_def];
				auto &p2def = rdef.ports[p2cfg.port_def];
				bool found_free = false;
				for (auto &prdef: p2def.val.wrprio) {
					// Check if the target matches.
					if (p1def.val.names[0] != prdef.val)
						continue;
					// Okay, we can use this cap.
					MemConfig cfg2 = cfg;
					if (wrport_opts_applied(cfg2, p2idx, prdef))
						found_free = true;
					else if (!apply_wrport_opts(cfg2, p2idx, prdef))
						continue;
					new_cfgs.push_back(cfg2);
				}
				if (!found_free) {
					// If no cap was found, or the cap found had a splitting
					// cost to it, consider emulation as well.
					p2cfg.emu_prio.push_back(p1idx);
					new_cfgs.push_back(cfg);
				}
			}
			cfgs = new_cfgs;
		}
	}
}

// Determine where to add soft init value logic.
void MemMapping::handle_rd_init() {
	for (int pidx = 0; pidx < GetSize(mem.rd_ports); pidx++) {
		auto &port = mem.rd_ports[pidx];
		// Only sync ports are relevant.
		if (!port.clk_enable)
			continue;
		// Skip ports with no init value.
		if (port.init_value.is_fully_undef())
			continue;
		MemConfigs new_cfgs;
		for (auto &cfg: cfgs) {
			auto &pcfg = cfg.rd_ports[pidx];
			auto &rdef = lib.ram_defs[cfg.ram_def];
			auto &pdef = rdef.ports[pcfg.port_def];
			// If emulated by async port or we already emulate CE, init will be
			// included for free.
			if (pcfg.emu_sync || pcfg.emu_en) {
				new_cfgs.push_back(cfg);
				continue;
			}
			// Otherwise, find a cap.
			bool found_free = false;
			for (auto &rstdef: pdef.val.rdrstval) {
				if (rstdef.val.kind != ResetKind::Init)
					continue;
				MemConfig cfg2 = cfg;
				auto &pcfg2 = cfg2.rd_ports[pidx];
				if (!apply_rstval(pcfg2, rstdef.val, port.init_value))
					continue;
				if (rdport_opts_applied(cfg2, pidx, rstdef))
					found_free = true;
				else if (!apply_rdport_opts(cfg2, pidx, rstdef))
					continue;
				new_cfgs.push_back(cfg2);
			}
			if (!found_free) {
				// If no cap was found, or the cap found had a splitting
				// cost to it, consider emulation as well.
				pcfg.emu_init = true;
				new_cfgs.push_back(cfg);
			}
		}
		cfgs = new_cfgs;
	}
}

// Determine where to add soft async reset logic.
void MemMapping::handle_rd_arst() {
	for (int pidx = 0; pidx < GetSize(mem.rd_ports); pidx++) {
		auto &port = mem.rd_ports[pidx];
		// Only sync ports are relevant.
		if (!port.clk_enable)
			continue;
		// Skip ports with no async reset.
		if (port.arst == State::S0)
			continue;
		if (port.arst_value.is_fully_undef())
			continue;
		MemConfigs new_cfgs;
		for (auto &cfg: cfgs) {
			auto &pcfg = cfg.rd_ports[pidx];
			auto &rdef = lib.ram_defs[cfg.ram_def];
			auto &pdef = rdef.ports[pcfg.port_def];
			// If emulated by async port or we already emulate CE, reset will be
			// included for free.
			if (pcfg.emu_sync || pcfg.emu_en) {
				new_cfgs.push_back(cfg);
				continue;
			}
			// Otherwise, find a cap.
			bool found_free = false;
			for (auto &rstdef: pdef.val.rdrstval) {
				if (rstdef.val.kind != ResetKind::Async)
					continue;
				MemConfig cfg2 = cfg;
				auto &pcfg2 = cfg2.rd_ports[pidx];
				if (!apply_rstval(pcfg2, rstdef.val, port.arst_value))
					continue;
				if (rdport_opts_applied(cfg2, pidx, rstdef))
					found_free = true;
				else if (!apply_rdport_opts(cfg2, pidx, rstdef))
					continue;
				new_cfgs.push_back(cfg2);
			}
			if (!found_free) {
				// If no cap was found, or the cap found had a splitting
				// cost to it, consider emulation as well.
				pcfg.emu_arst = true;
				new_cfgs.push_back(cfg);
			}
		}
		cfgs = new_cfgs;
	}
}

// Determine where to add soft sync reset logic.
void MemMapping::handle_rd_srst() {
	for (int pidx = 0; pidx < GetSize(mem.rd_ports); pidx++) {
		auto &port = mem.rd_ports[pidx];
		// Only sync ports are relevant.
		if (!port.clk_enable)
			continue;
		// Skip ports with no async reset.
		if (port.srst == State::S0)
			continue;
		if (port.srst_value.is_fully_undef())
			continue;
		MemConfigs new_cfgs;
		for (auto &cfg: cfgs) {
			auto &pcfg = cfg.rd_ports[pidx];
			auto &rdef = lib.ram_defs[cfg.ram_def];
			auto &pdef = rdef.ports[pcfg.port_def];
			// If emulated by async port or we already emulate CE, reset will be
			// included for free.
			if (pcfg.emu_sync || pcfg.emu_en) {
				new_cfgs.push_back(cfg);
				continue;
			}
			// Otherwise, find a cap.
			bool found_free = false;
			for (auto &rstdef: pdef.val.rdrstval) {
				if (rstdef.val.kind != ResetKind::Sync)
					continue;
				MemConfig cfg2 = cfg;
				auto &pcfg2 = cfg2.rd_ports[pidx];
				if (!apply_rstval(pcfg2, rstdef.val, port.srst_value))
					continue;
				if (rdport_opts_applied(cfg2, pidx, rstdef))
					found_free = true;
				else if (!apply_rdport_opts(cfg2, pidx, rstdef))
					continue;
				// If enable is in use, need to make sure the relative priority of
				// enable and srst is correct.  Otherwise, proceed immediately.
				if (port.en == State::S1) {
					new_cfgs.push_back(cfg2);
				} else {
					for (auto &mdef: pdef.val.rdsrstmode) {
						// Any value of the option is usable, at worst we'll emulate the priority.
						MemConfig cfg3 = cfg2;
						auto &pcfg3 = cfg3.rd_ports[pidx];
						if (mdef.val == SrstKind::SrstOverEn && port.ce_over_srst)
							pcfg3.emu_srst_en_prio = true;
						if (mdef.val == SrstKind::EnOverSrst && !port.ce_over_srst)
							pcfg3.emu_srst_en_prio = true;
						if (!apply_rdport_opts(cfg3, pidx, mdef))
							continue;
						new_cfgs.push_back(cfg3);
					}
				}
			}
			if (!found_free) {
				// If no cap was found, or the cap found had a splitting
				// cost to it, consider emulation as well.
				pcfg.emu_srst = true;
				new_cfgs.push_back(cfg);
			}
		}
		cfgs = new_cfgs;
	}
}

void MemMapping::score_emu_ports() {
	for (auto &cfg: cfgs) {
		auto &rdef = lib.ram_defs[cfg.ram_def];
		std::vector<int> port_usage_wr(rdef.ports.size());
		std::vector<int> port_usage_rd(rdef.ports.size());
		int score = 0;
		// 3 points for every write port if we need to do read-first emulation.
		if (cfg.emu_read_first)
			score += 3 * GetSize(cfg.wr_ports);
		for (auto &pcfg: cfg.wr_ports) {
			// 1 point for every priority relation we need to fix up.
			// This is just a gate for every distinct wren pair.
			score += GetSize(pcfg.emu_prio);
			port_usage_wr[pcfg.port_def]++;
		}
		for (auto &pcfg: cfg.rd_ports) {
			// 3 points for every soft transparency logic instance.  This involves
			// registers and other major mess.
			score += 3 * GetSize(pcfg.emu_trans);
			// 3 points for CE soft logic.  Likewise involves registers.
			// If we already do this, subsumes any init/srst/arst emulation.
			if (pcfg.emu_en)
				score += 3;
			// 2 points for soft init value / reset logic: involves single bit
			// register and some muxes.
			if (pcfg.emu_init)
				score += 2;
			if (pcfg.emu_arst)
				score += 2;
			if (pcfg.emu_srst)
				score += 2;
			// 1 point for wrong srst/en priority (fixed with a single gate).
			if (pcfg.emu_srst_en_prio)
				score++;
			// 1 point for every non-shared read port used, as a tiebreaker
			// to prefer single-port configs.
			if (pcfg.wr_port == -1) {
				score++;
				port_usage_rd[pcfg.port_def]++;
			}
		}
		cfg.score_emu = score;
		int repl_port = 1;
		for (int i = 0; i < GetSize(rdef.ports); i++) {
			int space = GetSize(rdef.ports[i].val.names) - port_usage_wr[i];
			log_assert(space >= 0);
			if (port_usage_rd[i] > 0) {
				log_assert(space > 0);
				int usage = port_usage_rd[i];
				int cur = (usage + space - 1) / space;
				if (cur > repl_port)
					repl_port = cur;
			}
		}
		cfg.repl_port = repl_port;
	}
}

void MemMapping::handle_geom_split() {
	// Split dims.
	MemConfigs new_cfgs;
	for (auto &cfg: cfgs) {
		auto &rdef = lib.ram_defs[cfg.ram_def];
		for (int didx = 0; didx < GetSize(rdef.dims); didx++) {
			auto &ddef = rdef.dims[didx];
			MemConfig cfg2 = cfg;
			if (!apply_opts(cfg2.opts, ddef.opts))
				continue;
			cfg2.dims_def = didx;
			new_cfgs.push_back(cfg2);
		}
	}
	cfgs = new_cfgs;
	// Split byte.
	new_cfgs.clear();
	for (auto &cfg: cfgs) {
		auto &rdef = lib.ram_defs[cfg.ram_def];
		bool found_free = false;
		for (auto &bdef: rdef.byte) {
			MemConfig cfg2 = cfg;
			if (opts_applied(cfg2.opts, bdef.opts))
				found_free = true;
			else if (!apply_opts(cfg2.opts, bdef.opts))
				continue;
			cfg2.byte = bdef.val;
			new_cfgs.push_back(cfg2);
		}
		if (!found_free) {
			cfg.byte = 0;
			new_cfgs.push_back(cfg);
		}
	}
	cfgs = new_cfgs;
	// Split width.
	for (int pidx = 0; pidx < GetSize(mem.wr_ports); pidx++) {
		MemConfigs new_cfgs;
		for (auto &cfg: cfgs) {
			auto &pcfg = cfg.wr_ports[pidx];
			auto &rdef = lib.ram_defs[cfg.ram_def];
			auto &pdef = rdef.ports[pcfg.port_def];
			for (int widx = 0; widx < GetSize(pdef.val.width); widx++) {
				auto &wdef = pdef.val.width[widx];
				MemConfig cfg2 = cfg;
				auto &pcfg2 = cfg2.wr_ports[pidx];
				if (!apply_wrport_opts(cfg2, pidx, wdef))
					continue;
				pcfg2.width_def = widx;
				new_cfgs.push_back(cfg2);
			}
		}
		cfgs = new_cfgs;
	}
	for (int pidx = 0; pidx < GetSize(mem.rd_ports); pidx++) {
		MemConfigs new_cfgs;
		for (auto &cfg: cfgs) {
			auto &pcfg = cfg.rd_ports[pidx];
			if (pcfg.wr_port != -1) {
				pcfg.width_def = cfg.wr_ports[pcfg.wr_port].width_def;
				new_cfgs.push_back(cfg);
				continue;
			}
			auto &rdef = lib.ram_defs[cfg.ram_def];
			auto &pdef = rdef.ports[pcfg.port_def];
			for (int widx = 0; widx < GetSize(pdef.val.width); widx++) {
				auto &wdef = pdef.val.width[widx];
				MemConfig cfg2 = cfg;
				auto &pcfg2 = cfg2.rd_ports[pidx];
				if (!apply_rdport_opts(cfg2, pidx, wdef))
					continue;
				pcfg2.width_def = widx;
				new_cfgs.push_back(cfg2);
			}
		}
		cfgs = new_cfgs;
	}
}

bool same_geom(const MemConfig &a, const MemConfig &b)
{
	if (a.ram_def != b.ram_def)
		return false;
	if (a.dims_def != b.dims_def)
		return false;
	if (a.byte != b.byte)
		return false;
	for (int i = 0; i < GetSize(a.wr_ports); i++) {
		auto &pa = a.wr_ports[i];
		auto &pb = b.wr_ports[i];
		if (pa.rd_port != pb.rd_port)
			return false;
		if (pa.port_def != pb.port_def)
			return false;
		if (pa.width_def != pb.width_def)
			return false;
	}
	for (int i = 0; i < GetSize(a.rd_ports); i++) {
		auto &pa = a.rd_ports[i];
		auto &pb = b.rd_ports[i];
		if (pa.wr_port != pb.wr_port)
			return false;
		if (pa.port_def != pb.port_def)
			return false;
		if (pa.width_def != pb.width_def)
			return false;
	}
	return true;
}

void MemMapping::prune_pre_geom() {
	std::vector<bool> keep;
	for (int i = 0; i < GetSize(cfgs); i++) {
		for (int j = 0; j < i; j++) {
			if (!keep[j])
				continue;
			if (same_geom(cfgs[i], cfgs[j])) {
				if (cfgs[i].score_emu < cfgs[j].score_emu) {
					keep[i] = false;
					keep.push_back(true);
				} else {
					keep.push_back(false);
				}
				goto found;
			}
		}
		keep.push_back(true);
found:;
	}
	MemConfigs new_cfgs;
	for (int i = 0; i < GetSize(cfgs); i++)
		if (keep[i])
			new_cfgs.push_back(cfgs[i]);
	cfgs = new_cfgs;
}

std::pair<int, int> xlat_width_range(const MemoryDimsDef &dims, const std::vector<int> &widths) {
	if (widths.empty())
		return {0, GetSize(dims.dbits) - 1};
	for (int i = 0; i < GetSize(dims.dbits); i++) {
		if (dims.dbits[i] == widths[0]) {
			return {i, i + GetSize(widths) - 1};
		}
	}
	abort();
}

void MemMapping::handle_geom() {
	// First, create a set of "byte boundaries": the bit positions in source memory word
	// that have write enable different from the previous bit in any write port.
	// Bit 0 is considered to be a byte boundary as well.
	std::vector<bool> byte_boundary(mem.width, false);
	byte_boundary[0] = true;
	for (auto &port: mem.wr_ports) {
		for (int sub = 0; sub < port.wide_log2; sub++) {
			for (int i = 1; i < mem.width; i++) {
				int pos = sub * mem.width + i;
				if (port.en[pos] != port.en[pos-1])
					byte_boundary[i] = true;
			}
		}
	}
	std::vector<int> wren_size;
	for (auto &port: mem.wr_ports) {
		SigSpec en = port.en;
		en.sort_and_unify();
		wren_size.push_back(GetSize(en));
	}
	for (auto &cfg: cfgs) {
		auto &rdef = lib.ram_defs[cfg.ram_def];
		auto &dims = rdef.dims[cfg.dims_def].val;
		bool got_config = false;
		int best_cost = 0;
		// First, convert WidthDef and byte width into a more convenient form.
		std::vector<std::pair<int, int>> wr_width_range;
		std::vector<std::pair<int, int>> rd_width_range;
		for (auto &pcfg: cfg.wr_ports) {
			auto &pdef = rdef.ports[pcfg.port_def].val;
			auto &wdef = pdef.width[pcfg.width_def].val;
			wr_width_range.push_back(xlat_width_range(dims, wdef.wr_widths));
		}
		for (auto &pcfg: cfg.rd_ports) {
			auto &pdef = rdef.ports[pcfg.port_def].val;
			auto &wdef = pdef.width[pcfg.width_def].val;
			rd_width_range.push_back(xlat_width_range(dims, wdef.tied ? wdef.wr_widths : wdef.rd_widths));
		}
		int byte_width_log2 = 0;
		for (int i = 0; i < GetSize(dims.dbits); i++)
			if (cfg.byte >= dims.dbits[i])
				byte_width_log2 = i;
		if (cfg.byte == 0)
			byte_width_log2 = GetSize(dims.dbits) - 1;
		// Second, determine which of the source address bits involved in wide ports
		// are "uniform".  Bits are considered uniform if, when a port is widened through
		// them, the write enables are the same for both values of the bit.
		int max_wr_wide_log2 = 0;
		for (auto &port: mem.wr_ports)
			if (port.wide_log2 > max_wr_wide_log2)
				max_wr_wide_log2 = port.wide_log2;
		int max_wide_log2 = max_wr_wide_log2;
		for (auto &port: mem.rd_ports)
			if (port.wide_log2 > max_wide_log2)
				max_wide_log2 = port.wide_log2;
		int wide_nu_start = max_wide_log2;
		int wide_nu_end = max_wr_wide_log2;
		for (int i = 0; i < GetSize(mem.wr_ports); i++) {
			auto &port = mem.wr_ports[i];
			auto &pcfg = cfg.wr_ports[i];
			auto &pdef = rdef.ports[pcfg.port_def].val;
			auto &wdef = pdef.width[pcfg.width_def].val;
			for (int j = 0; j < port.wide_log2; j++) {
				bool uniform = true;
				// If write enables don't match, mark bit as non-uniform.
				for (int k = 0; k < (1 << port.wide_log2); k += 2 << j)
					if (port.en.extract(k * mem.width, mem.width << j) != port.en.extract((k + (1 << j)) * mem.width, mem.width << j))
						uniform = false;
				if (!uniform) {
					if (j < wide_nu_start)
						wide_nu_start = j;
					break;
				}
			}
			if (wdef.tied && pcfg.rd_port != -1) {
				// If:
				//
				// - the write port is merged with a read port
				// - the read port is wider than the write port
				// - read and write widths are tied
				//
				// then we will have to artificially widen the write
				// port to the width of the read port, and emulate
				// a narrower write path by use of write enables,
				// which will definitely be non-uniform over the added
				// bits.
				auto &rport = mem.rd_ports[pcfg.rd_port];
				if (rport.wide_log2 > port.wide_log2) {
					if (port.wide_log2 < wide_nu_start)
						wide_nu_start = port.wide_log2;
					if (rport.wide_log2 > wide_nu_end)
						wide_nu_end = rport.wide_log2;
				}
			}
		}
		// Determine lowest reasonable base width.
		int start_base = GetSize(dims.dbits) - 1;
		for (auto &x: wr_width_range)
			if (x.first < start_base)
				start_base = x.first;
		for (auto &x: rd_width_range)
			if (x.first < start_base)
				start_base = x.first;
		// Iterate over base widths.
		for (int base_width_log2 = start_base; base_width_log2 < GetSize(dims.dbits); base_width_log2++) {
			// Now, see how many data bits we actually have available.
			// This is usually dbits[base_width_log2], but could be smaller if we
			// ran afoul of a max width limitation.  Configurations where this
			// happens are not useful, unless we need it to satisfy a *minimum*
			// width limitation.
			int unit_width_log2 = base_width_log2;
			for (auto &x: wr_width_range)
				if (unit_width_log2 > x.second)
					unit_width_log2 = x.second;
			for (auto &x: rd_width_range)
				if (unit_width_log2 > x.second)
					unit_width_log2 = x.second;
			if (unit_width_log2 != base_width_log2 && got_config)
				break;
			int unit_width = dims.dbits[unit_width_log2];
			// Also determine effective byte width (the granularity of write enables).
			int effective_byte = cfg.byte;
			if (cfg.byte == 0 || cfg.byte > unit_width)
				effective_byte = unit_width;
			if (mem.wr_ports.empty())
				effective_byte = 1;
			log_assert(unit_width % effective_byte == 0);
			// Create the swizzle pattern.
			std::vector<int> swizzle;
			for (int i = 0; i < mem.width; i++) {
				if (byte_boundary[i])
					while (GetSize(swizzle) % effective_byte)
						swizzle.push_back(-1);
				swizzle.push_back(i);
			}
			while (GetSize(swizzle) % effective_byte)
				swizzle.push_back(-1);
			// Now evaluate the configuration, then keep adding more hard wide bits
			// and evaluating.
			int hard_wide_mask = 0;
			int hard_wide_num = 0;
			bool byte_failed = false;
			while (1) {
				// Check if all min width constraints are satisfied.
				// Only check these constraints for write ports with width below
				// byte width — for other ports, we can emulate narrow width with
				// a larger one.
				bool min_width_ok = true;
				int min_width_bit = wide_nu_start;
				for (int pidx = 0; pidx < GetSize(mem.wr_ports); pidx++) {
					auto &port = mem.wr_ports[pidx];
					int w = base_width_log2;
					for (int i = 0; i < port.wide_log2; i++)
						if (hard_wide_mask & 1 << i)
							w++;
					if (w < wr_width_range[pidx].first && w < byte_width_log2) {
						min_width_ok = false;
						if (min_width_bit > port.wide_log2)
							min_width_bit = port.wide_log2;
					}
				}
				if (min_width_ok) {
					int emu_wide_bits = max_wide_log2 - hard_wide_num;
					int mult_wide = 1 << emu_wide_bits;
					int addrs = 1 << (dims.abits - base_width_log2 + emu_wide_bits);
					int min_addr = mem.start_offset / addrs;
					int max_addr = (mem.start_offset + mem.size - 1) / addrs;
					int mult_a = max_addr - min_addr + 1;
					int bits = mult_a * mult_wide * GetSize(swizzle);
					int repl = (bits + unit_width - 1) / unit_width;
					int score_demux = 0;
					for (int i = 0; i < GetSize(mem.wr_ports); i++) {
						auto &port = mem.wr_ports[i];
						int w = emu_wide_bits;
						for (int i = 0; i < port.wide_log2; i++)
							if (!(hard_wide_mask & 1 << i))
								w--;
						if (w || mult_a != 1)
							score_demux += (mult_a << w) * wren_size[i];
					}
					int score_mux = 0;
					for (auto &port: mem.rd_ports) {
						int w = emu_wide_bits;
						for (int i = 0; i < port.wide_log2; i++)
							if (!(hard_wide_mask & 1 << i))
								w--;
						score_mux += ((mult_a << w) - 1) * GetSize(port.data);
					}
					double cost = dims.cost * repl * cfg.repl_port;
					cost += score_mux * FACTOR_MUX;
					cost += score_demux * FACTOR_DEMUX;
					cost += cfg.score_emu * FACTOR_EMU;
					if (!got_config || cost < best_cost) {
						cfg.base_width_log2 = base_width_log2;
						cfg.unit_width_log2 = unit_width_log2;
						cfg.swizzle = swizzle;
						cfg.hard_wide_mask = hard_wide_mask;
						cfg.emu_wide_mask = ((1 << max_wide_log2) - 1) & ~hard_wide_mask;
						cfg.repl_d = repl;
						cfg.score_demux = score_demux;
						cfg.score_mux = score_mux;
						cfg.cost = cost;
						best_cost = cost;
						got_config = true;
					}
				}
				if (dims.tied)
					break;
				// Now, pick the next bit to add to the hard wide mask.
next_hw:
				int scan_from;
				int scan_to;
				bool retry = false;
				if (!min_width_ok) {
					// If we still haven't met the minimum width limits,
					// add the highest one that will be useful for working
					// towards all unmet limits.
					scan_from = min_width_bit;
					scan_to = 0;
					// If the relevant write port is not wide, it's impossible.
				} else if (byte_failed) {
					// If we already failed with uniformly-written bits only,
					// go with uniform bits that are only involved in reads.
					scan_from = max_wide_log2;
					scan_to = wide_nu_end;
				} else if (base_width_log2 + hard_wide_num < byte_width_log2) {
					// If we still need uniform bits, prefer the low ones.
					scan_from = wide_nu_start;
					scan_to = 0;
					retry = true;
				} else {
					scan_from = max_wide_log2;
					scan_to = 0;
				}
				int bit = scan_from - 1;
				while (1) {
					if (bit < scan_to) {
hw_bit_failed:
						if (retry) {
							byte_failed = true;
							goto next_hw;
						} else {
							goto bw_done;
						}
					}
					if (!(hard_wide_mask & 1 << bit))
						break;
					bit--;
				}
				int new_hw_mask = hard_wide_mask | 1 << bit;
				// Check if all max width constraints are satisfied.
				for (int pidx = 0; pidx < GetSize(mem.wr_ports); pidx++) {
					auto &port = mem.wr_ports[pidx];
					int w = base_width_log2;
					for (int i = 0; i < port.wide_log2; i++)
						if (new_hw_mask & 1 << i)
							w++;
					if (w > wr_width_range[pidx].second) {
						goto hw_bit_failed;
					}
				}
				for (int pidx = 0; pidx < GetSize(mem.rd_ports); pidx++) {
					auto &port = mem.rd_ports[pidx];
					int w = base_width_log2;
					for (int i = 0; i < port.wide_log2; i++)
						if (new_hw_mask & 1 << i)
							w++;
					if (w > rd_width_range[pidx].second) {
						goto hw_bit_failed;
					}
				}
				// Bit ok, commit.
				hard_wide_mask = new_hw_mask;
				hard_wide_num++;
			}
bw_done:;
		}
		log_assert(got_config);
	}
}

void MemMapping::prune_post_geom() {
	std::vector<bool> keep;
	dict<std::string, int> rsrc;
	for (int i = 0; i < GetSize(cfgs); i++) {
		auto &cfg = cfgs[i];
		auto &rdef = lib.ram_defs[cfg.ram_def];
		auto &dims = rdef.dims[cfg.dims_def].val;
		std::string key = dims.resource_name;
		if (key.empty()) {
			switch (rdef.kind) {
				case RamKind::Distributed:
					key = "[distributed]";
					break;
				case RamKind::Block:
					key = "[block]";
					break;
				case RamKind::Huge:
					key = "[huge]";
					break;
				default:
					break;
			}
		}
		auto it = rsrc.find(key);
		if (it == rsrc.end()) {
			rsrc[key] = i;
			keep.push_back(true);
		} else {
			auto &ocfg = cfgs[it->second];
			if (cfg.cost < ocfg.cost) {
				keep[it->second] = false;
				it->second = i;
				keep.push_back(true);
			} else {
				keep.push_back(false);
			}
			
		}
	}
	MemConfigs new_cfgs;
	for (int i = 0; i < GetSize(cfgs); i++)
		if (keep[i])
			new_cfgs.push_back(cfgs[i]);
	cfgs = new_cfgs;
}

Swizzle MemMapping::gen_swizzle(const MemConfig &cfg, int sw_wide_log2, int hw_wide_log2) {
	auto &rdef = lib.ram_defs[cfg.ram_def];
	auto &dims = rdef.dims[cfg.dims_def].val;
	Swizzle res;

	std::vector<int> emu_wide_bits;
	std::vector<int> hard_wide_bits;
	for (int i = 0; i < ceil_log2(mem.size); i++) {
		if (cfg.emu_wide_mask & 1 << i)
			emu_wide_bits.push_back(i);
		else if (GetSize(hard_wide_bits) < hw_wide_log2 - cfg.base_width_log2)
			hard_wide_bits.push_back(i);
	}
	for (int x : hard_wide_bits)
		if (x >= sw_wide_log2)
			res.addr_mux_bits.push_back(x);
	for (int x : emu_wide_bits)
		if (x >= sw_wide_log2)
			res.addr_mux_bits.push_back(x);

	int addr_shift = dims.abits - cfg.base_width_log2 + GetSize(emu_wide_bits);
	int addr_start = mem.start_offset & ~((1 << addr_shift) - 1);
	int addr_end = ((mem.start_offset + mem.size - 1) | ((1 << addr_shift) - 1)) + 1;
	int hnum = (addr_end - addr_start) >> addr_shift;
	int unit_width = dims.dbits[cfg.unit_width_log2];
	res.addr_shift = addr_shift;

	for (int rd = 0; rd < cfg.repl_d; rd++) {
		std::vector<SwizzleBit> bits(dims.dbits[hw_wide_log2]);
		for (auto &bit: bits)
			bit.valid = false;
		res.bits.push_back(bits);
	}

	for (int hi = 0; hi < hnum; hi++) {
		for (int ewi = 0; ewi < (1 << GetSize(emu_wide_bits)); ewi++) {
			for (int hwi = 0; hwi < (1 << GetSize(hard_wide_bits)); hwi++) {
				int mux_idx = 0;
				int sub = 0;
				int mib = 0;
				int hbit_base = 0;
				for (int i = 0; i < GetSize(hard_wide_bits); i++) {
					if (hard_wide_bits[i] < sw_wide_log2) {
						if (hwi & 1 << i)
							sub |= 1 << hard_wide_bits[i];
					} else {
						if (hwi & 1 << i)
							mux_idx |= 1 << mib;
						mib++;
					}
					if (hwi & 1 << i)
						hbit_base += dims.dbits[i + cfg.base_width_log2];
				}
				for (int i = 0; i < GetSize(emu_wide_bits); i++) {
					if (emu_wide_bits[i] < sw_wide_log2) {
						if (ewi & 1 << i)
							sub |= 1 << emu_wide_bits[i];
					} else {
						if (ewi & 1 << i)
							mux_idx |= 1 << mib;
						mib++;
					}
				}
				mux_idx |= hi << mib;
				int addr = addr_start + (hi << addr_shift);
				for (int i = 0; i < GetSize(res.addr_mux_bits); i++)
					if (mux_idx & 1 << i)
						addr += 1 << res.addr_mux_bits[i];
				for (int bit = 0; bit < GetSize(cfg.swizzle); bit++) {
					if (cfg.swizzle[bit] == -1)
						continue;
					int rbit = bit + GetSize(cfg.swizzle) * (ewi + (hi << GetSize(emu_wide_bits)));
					int rep = rbit / unit_width;
					int hbit = hbit_base + rbit % unit_width;
					auto &swz = res.bits[rep][hbit];
					swz.valid = true;
					swz.addr = addr;
					swz.mux_idx = mux_idx;
					swz.bit = cfg.swizzle[bit] + sub * mem.width;
				}
			}
		}
	}

	return res;
}

void MemMapping::emit(const MemConfig &cfg) {
	auto &rdef = lib.ram_defs[cfg.ram_def];
	auto &dims = rdef.dims[cfg.dims_def].val;
	log("mapping memory %s.%s via %s\n", log_id(mem.module->name), log_id(mem.memid), log_id(rdef.id));
	// First, handle emulations.
	if (cfg.emu_read_first)
		mem.emulate_read_first(&worker.initvals);
	for (int pidx = 0; pidx < GetSize(mem.rd_ports); pidx++) {
		auto &pcfg = cfg.rd_ports[pidx];
		auto &port = mem.rd_ports[pidx];
		if (pcfg.emu_sync)
			mem.extract_rdff(pidx, &worker.initvals);
		else if (pcfg.emu_en)
			mem.emulate_rden(pidx, &worker.initvals);
		else {
			if (pcfg.emu_srst_en_prio) {
				if (port.ce_over_srst)
					mem.emulate_rd_ce_over_srst(pidx);
				else
					mem.emulate_rd_srst_over_ce(pidx);
			}
			mem.emulate_reset(pidx, pcfg.emu_init, pcfg.emu_arst, pcfg.emu_srst, &worker.initvals);
		}
	}
	for (int pidx = 0; pidx < GetSize(mem.wr_ports); pidx++) {
		auto &pcfg = cfg.wr_ports[pidx];
		for (int opidx: pcfg.emu_prio) {
			mem.emulate_priority(opidx, pidx, &worker.initvals);
		}
	}
	for (int pidx = 0; pidx < GetSize(mem.rd_ports); pidx++) {
		auto &port = mem.rd_ports[pidx];
		auto &pcfg = cfg.rd_ports[pidx];
		for (int opidx: pcfg.emu_trans) {
			// The port may no longer be transparent due to transparency being
			// nuked as part of emu_sync or emu_prio.
			if (port.transparency_mask[opidx])
				mem.emulate_transparency(opidx, pidx, &worker.initvals);
		}
	}

	std::vector<std::vector<Cell *>> cells(cfg.repl_port);
	// tgt (repl, port group, port) -> mem (wr port, rd port), where -1 means no port.
	std::vector<std::vector<std::vector<std::pair<int, int>>>> ports(cfg.repl_port);
	for (int i = 0; i < cfg.repl_port; i++)
		ports[i].resize(rdef.ports.size());
	for (int i = 0; i < GetSize(cfg.wr_ports); i++) {
		auto &pcfg = cfg.wr_ports[i];
		for (int j = 0; j < cfg.repl_port; j++) {
			if (j == 0) {
				ports[j][pcfg.port_def].push_back({i, pcfg.rd_port});
			} else {
				ports[j][pcfg.port_def].push_back({i, -1});
			}
		}
	}
	for (int i = 0; i < GetSize(cfg.rd_ports); i++) {
		auto &pcfg = cfg.rd_ports[i];
		if (pcfg.wr_port != -1)
			continue;
		auto &pdef = rdef.ports[pcfg.port_def].val;
		int j = 0;
		while (GetSize(ports[j][pcfg.port_def]) >= GetSize(pdef.names))
			j++;
		ports[j][pcfg.port_def].push_back({-1, i});
	}

	Swizzle init_swz = gen_swizzle(cfg, 0, GetSize(dims.dbits) - 1);
	Const init_data = mem.get_init_data();

	std::vector<int> hw_addr_swizzle;
	for (int i = 0; i < cfg.base_width_log2; i++)
		hw_addr_swizzle.push_back(-1);
	for (int i = 0; i < init_swz.addr_shift; i++)
		if (!(cfg.emu_wide_mask & 1 << i))
			hw_addr_swizzle.push_back(i);
	log_assert(GetSize(hw_addr_swizzle) == dims.abits);

	for (int rp = 0; rp < cfg.repl_port; rp++) {
		for (int rd = 0; rd < cfg.repl_d; rd++) {
			Cell *cell = mem.module->addCell(stringf("%s.%d.%d", mem.memid.c_str(), rp, rd), rdef.id);
			cell->setParam(ID::ABITS, dims.abits);
			cell->setParam(ID::BYTE, cfg.byte);
			if (dims.tied)
				cell->setParam(ID::WIDTH, dims.dbits[cfg.base_width_log2]);
			for (auto &it: cfg.opts)
				cell->setParam(stringf("\\OPTION_%s", it.first.c_str()), it.second);
			for (auto &it: cfg.clocks_anyedge) {
				cell->setParam(stringf("\\CLKPOL_%s", it.first.c_str()), it.second.second);
				cell->setPort(stringf("\\CLK_%s", it.first.c_str()), it.second.first);
			}
			for (auto &it: cfg.clocks_pnedge) {
				SigSpec sig = it.second.first;
				if (it.second.second)
					sig = mem.module->Not(NEW_ID, sig);
				cell->setPort(stringf("\\CLK_%s", it.first.c_str()), sig);
			}
			std::vector<State> initval;
			for (int hwa = 0; hwa < (1 << dims.abits); hwa += 1 << (GetSize(dims.dbits) - 1)) {
				for (auto &bit: init_swz.bits[rd]) {
					if (!bit.valid) {
						initval.push_back(State::Sx);
					} else {
						int addr = bit.addr;
						for (int i = GetSize(dims.dbits) - 1; i < dims.abits; i++)
							if (hwa & 1 << i)
								addr += 1 << hw_addr_swizzle[i];
						if (addr >= mem.start_offset && addr < mem.start_offset + mem.size)
							initval.push_back(init_data.bits[(addr - mem.start_offset) * mem.width + bit.bit]);
						else
							initval.push_back(State::Sx);
					}
				}
			}
			cell->setParam(ID::INIT, initval);
			cells[rp].push_back(cell);
		}
		for (int pgi = 0; pgi < GetSize(rdef.ports); pgi++) {
			auto &pdef = rdef.ports[pgi].val;
			for (int pi = 0; pi < GetSize(pdef.names); pi++) {
				auto &pname = pdef.names[pi];
				if (pi < GetSize(ports[rp][pgi])) {
					int wpidx = ports[rp][pgi][pi].first;
					int rpidx = ports[rp][pgi][pi].second;
					for (auto cell: cells[rp]) {
						cell->setParam(stringf("\\PORT_%s_USED", pname.c_str()), true);
						if (pdef.kind == PortKind::Srsw || pdef.kind == PortKind::Arsw) {
							cell->setParam(stringf("\\PORT_%s_WR_USED", pname.c_str()), wpidx != -1);
							cell->setParam(stringf("\\PORT_%s_RD_USED", pname.c_str()), rpidx != -1);
						}
					}
					SigSpec addr;
					int width_def;
					int wide_log2 = 0, wr_wide_log2 = 0, rd_wide_log2 = 0;
					SigSpec clk = State::S0;
					bool clk_pol = true;
					ClkPolKind clkpol_kind = ClkPolKind::Posedge;
					if (wpidx != -1) {
						auto &wpcfg = cfg.wr_ports[wpidx];
						auto &wport = mem.wr_ports[wpidx];
						for (auto &it: wpcfg.portopts)
							for (auto cell: cells[rp])
								cell->setParam(stringf("\\PORT_%s_OPTION_%s", pname.c_str(), it.first.c_str()), it.second);
						clk = wport.clk;
						clk_pol = wport.clk_polarity;
						clkpol_kind = wpcfg.clkpol_kind;
						addr = wport.addr;
						width_def = wpcfg.width_def;
						wide_log2 = wr_wide_log2 = wport.wide_log2;
						if (rpidx != -1) {
							auto &rport = mem.wr_ports[wpidx];
							rd_wide_log2 = rport.wide_log2;
							if (rd_wide_log2 > wr_wide_log2)
								wide_log2 = rd_wide_log2;
							else
								addr = rport.addr;
						}
					} else {
						auto &rpcfg = cfg.rd_ports[rpidx];
						auto &rport = mem.rd_ports[rpidx];
						for (auto &it: rpcfg.portopts)
							for (auto cell: cells[rp])
								cell->setParam(stringf("\\PORT_%s_OPTION_%s", pname.c_str(), it.first.c_str()), it.second);
						if (rport.clk_enable) {
							clk = rport.clk;
							clk_pol = rport.clk_polarity;
							clkpol_kind = rpcfg.clkpol_kind;
						}
						addr = rport.addr;
						width_def = rpcfg.width_def;
						wide_log2 = rd_wide_log2 = rport.wide_log2;
					}
					addr = worker.sigmap_xmux(addr);
					if (pdef.kind != PortKind::Ar) {
						switch (clkpol_kind) {
							case ClkPolKind::Posedge:
								if (!clk_pol)
									clk = mem.module->Not(NEW_ID, clk);
								break;
							case ClkPolKind::Negedge:
								if (clk_pol)
									clk = mem.module->Not(NEW_ID, clk);
								break;
							case ClkPolKind::Anyedge:
								for (auto cell: cells[rp])
									cell->setParam(stringf("\\PORT_%s_CLKPOL", pname.c_str()), clk_pol);
						}
						for (auto cell: cells[rp])
							cell->setPort(stringf("\\PORT_%s_CLK", pname.c_str()), clk);
					}

					// Width determination.
					auto &wdef = pdef.width[width_def].val;
					auto wr_width_range = xlat_width_range(dims, wdef.wr_widths);
					auto rd_width_range = xlat_width_range(dims, wdef.rd_widths);
					if (wdef.tied) {
						rd_wide_log2 = wr_wide_log2 = wide_log2;
					}
					int hw_wr_wide_log2 = cfg.base_width_log2;
					for (int i = 0; i < wr_wide_log2; i++)
						if (cfg.hard_wide_mask & (1 << i))
							hw_wr_wide_log2++;
					if (hw_wr_wide_log2 < wr_width_range.first)
						hw_wr_wide_log2 = wr_width_range.first;
					if (hw_wr_wide_log2 > wr_width_range.second)
						hw_wr_wide_log2 = wr_width_range.second;
					int hw_rd_wide_log2 = cfg.base_width_log2;
					for (int i = 0; i < rd_wide_log2; i++)
						if (cfg.hard_wide_mask & (1 << i))
							hw_rd_wide_log2++;
					if (hw_rd_wide_log2 < rd_width_range.first)
						hw_rd_wide_log2 = rd_width_range.first;
					if (hw_rd_wide_log2 > rd_width_range.second)
						hw_rd_wide_log2 = rd_width_range.second;
					if (!dims.tied) {
						for (auto cell: cells[rp]) {
							if (wdef.tied) {
								cell->setParam(stringf("\\PORT_%s_WIDTH", pname.c_str()), dims.dbits[hw_wr_wide_log2]);
							} else {
								if (wpidx != -1)
									cell->setParam(stringf("\\PORT_%s_WR_WIDTH", pname.c_str()), dims.dbits[hw_wr_wide_log2]);
								if (rpidx != -1)
									cell->setParam(stringf("\\PORT_%s_RD_WIDTH", pname.c_str()), dims.dbits[hw_rd_wide_log2]);
							}
						}
					}

					// Address determination.
					SigSpec hw_addr;
					for (int x: hw_addr_swizzle) {
						if (x == -1 || x >= GetSize(addr))
							hw_addr.append(State::S0);
						else
							hw_addr.append(addr[x]);
					}
					for (int i = 0; i < hw_wr_wide_log2 && i < hw_rd_wide_log2; i++)
						hw_addr[i] = State::S0;
					for (auto cell: cells[rp])
						cell->setPort(stringf("\\PORT_%s_ADDR", pname.c_str()), hw_addr);

					if (wpidx != -1) {
						auto &wport = mem.wr_ports[wpidx];
						Swizzle port_swz = gen_swizzle(cfg, wport.wide_log2, hw_wr_wide_log2);
						int effective_byte = cfg.byte;
						if (effective_byte == 0 || effective_byte > dims.dbits[hw_wr_wide_log2])
							effective_byte = dims.dbits[hw_wr_wide_log2];
						std::vector<SigSpec> big_wren = mem.generate_demux(wpidx, port_swz.addr_shift, port_swz.addr_mux_bits);
						for (int rd = 0; rd < cfg.repl_d; rd++) {
							auto cell = cells[rp][rd];
							SigSpec hw_wdata;
							SigSpec hw_wren;
							for (auto &bit : port_swz.bits[rd]) {
								if (!bit.valid) {
									hw_wdata.append(State::Sx);
								} else {
									hw_wdata.append(wport.data[bit.bit]);
								}
							}
							for (int i = 0; i < GetSize(port_swz.bits[rd]); i += effective_byte) {
								auto &bit = port_swz.bits[rd][i];
								if (!bit.valid) {
									hw_wren.append(State::S0);
								} else {
									hw_wren.append(big_wren[bit.mux_idx][bit.bit]);
								}
							}
							cell->setPort(stringf("\\PORT_%s_WR_DATA", pname.c_str()), hw_wdata);
							cell->setPort(stringf("\\PORT_%s_WR_EN", pname.c_str()), hw_wren);
							cell->setParam(stringf("\\PORT_%s_WR_EN_WIDTH", pname.c_str()), GetSize(hw_wren));
						}
					}
					if (rpidx != -1) {
						auto &rpcfg = cfg.rd_ports[rpidx];
						auto &rport = mem.rd_ports[rpidx];
						Swizzle port_swz = gen_swizzle(cfg, rport.wide_log2, hw_rd_wide_log2);
						std::vector<SigSpec> big_rdata = mem.generate_mux(rpidx, port_swz.addr_shift, port_swz.addr_mux_bits);
						for (int rd = 0; rd < cfg.repl_d; rd++) {
							auto cell = cells[rp][rd];
							if (rpcfg.emit_en) {
								cell->setPort(stringf("\\PORT_%s_RD_EN", pname.c_str()), rport.en);
							}
							if (rport.arst != State::S0) {
								cell->setPort(stringf("\\PORT_%s_RD_ARST", pname.c_str()), rport.arst);
								cell->setParam(stringf("\\PORT_%s_RD_ARST_USED", pname.c_str()), true);
							}
							if (rport.srst != State::S0) {
								cell->setPort(stringf("\\PORT_%s_RD_SRST", pname.c_str()), rport.srst);
								cell->setParam(stringf("\\PORT_%s_RD_SRST_USED", pname.c_str()), true);
								cell->setParam(stringf("\\PORT_%s_RD_CE_OVER_SRST", pname.c_str()), rport.ce_over_srst);
							}
							for (auto &it: rpcfg.resetvals) {
								std::vector<State> val;
								for (auto &bit : port_swz.bits[rd]) {
									if (!bit.valid) {
										val.push_back(State::Sx);
									} else {
										val.push_back(it.second.bits[bit.bit]);
									}
								}
								cell->setParam(stringf("\\PORT_%s_%s", pname.c_str(), it.first.c_str()), val);
							}
							SigSpec hw_rdata = mem.module->addWire(NEW_ID, dims.dbits[hw_rd_wide_log2]);
							cell->setPort(stringf("\\PORT_%s_RD_DATA", pname.c_str()), hw_rdata);
							SigSpec lhs;
							SigSpec rhs;
							for (int i = 0; i < GetSize(hw_rdata); i++) {
								auto &bit = port_swz.bits[rd][i];
								if (bit.valid) {
									lhs.append(big_rdata[bit.mux_idx][bit.bit]);
									rhs.append(hw_rdata[i]);
								}
							}
							mem.module->connect(lhs, rhs);
						}
					}
				} else {
					for (auto cell: cells[rp])
						cell->setParam(stringf("\\PORT_%s_USED", pname.c_str()), false);
				}
			}
		}
	}
	mem.remove();
}

struct MemoryLibMapPass : public Pass {
	MemoryLibMapPass() : Pass("memory_libmap", "map memories to cells") { }
	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("    memory_libmap -lib <library_file> [-D <condition>] [selection]\n");
		log("\n");
		log("This pass takes a description of available RAM cell types and maps\n");
		log("all selected memories to one of them, or leaves them  to be mapped to FFs.\n");
		log("\n");
		log("  -lib <library_file>\n");
		log("    Selects a library file containing RAM cell definitions. This option\n");
		log("    can be passed more than once to select multiple libraries.\n");
		log("\n");
		log("  -D <condition>\n");
		log("    Enables a condition that can be checked within the library file\n");
		log("    to eg. select between slightly different hardware variants.\n");
		log("    This option can be passed any number of times.\n");
		log("\n");
	}
	void execute(std::vector<std::string> args, RTLIL::Design *design) override
	{
		std::vector<std::string> lib_files;
		pool<std::string> defines;
		PassOptions opts;
		opts.no_auto_distributed = false;
		opts.no_auto_block = false;
		opts.no_auto_huge = false;
		opts.debug_geom = false;
		log_header(design, "Executing MEMORY_LIBMAP pass (mapping memories to cells).\n");

		size_t argidx;
		for (argidx = 1; argidx < args.size(); argidx++) {
			if (args[argidx] == "-lib" && argidx+1 < args.size()) {
				lib_files.push_back(args[++argidx]);
				continue;
			}
			if (args[argidx] == "-D" && argidx+1 < args.size()) {
				defines.insert(args[++argidx]);
				continue;
			}
			if (args[argidx] == "-no-auto-distributed") {
				opts.no_auto_distributed = true;
				continue;
			}
			if (args[argidx] == "-no-auto-block") {
				opts.no_auto_block = true;
				continue;
			}
			if (args[argidx] == "-no-auto-huge") {
				opts.no_auto_huge = true;
				continue;
			}
			if (args[argidx] == "-debug-geom") {
				opts.debug_geom = true;
				continue;
			}
			break;
		}
		extra_args(args, argidx, design);

		Library lib(opts, defines);
		for (auto &file: lib_files) {
			Parser(file, lib);
		}
		lib.prepare();

		for (auto module : design->selected_modules()) {
			MapWorker worker(module);
			auto mems = Mem::get_selected_memories(module);
			for (auto &mem : mems)
			{
				MemMapping map(worker, mem, lib);
				int idx = -1;
				int best = map.logic_cost;
				if (!map.logic_ok) {
					if (map.cfgs.empty())
						log_error("no valid mapping found for memory %s.%s\n", log_id(module->name), log_id(mem.memid));
					idx = 0;
					best = map.cfgs[0].cost;
				}
				for (int i = 0; i < GetSize(map.cfgs); i++) {
					if (map.cfgs[i].cost < best) {
						idx = i;
						best = map.cfgs[i].cost;
					}
				}
				if (idx == -1) {
					log("using FF mapping for memory %s.%s\n", log_id(module->name), log_id(mem.memid));
				} else {
					map.emit(map.cfgs[idx]);
				}
			}
		}
	}
} MemoryLibMapPass;

PRIVATE_NAMESPACE_END
