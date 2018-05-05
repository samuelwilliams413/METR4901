#include <stdlib.h>                                                   /* QS: */
#include <stdio.h>                                                    /* QS: */
#include <ctype.h>                                                    /* QS: */
#include <string.h>                                                   /* QS: */
#include "extra.h"                                                    /* QS: */
typedef unsigned char BOOL;                                           /* QS: */
                                                                      /* QS: */
#define FALSE 0                                                  /* QS:a0w5z */
#define TRUE !FALSE                                              /* QS:a1d1x */
enum ERRS {                                                           /* QS: */
    OK = 0,                                                      /* QS:a2k6u */
    USAGE = 1,                                                   /* QS:a3r1s */
    DIM = 7,                                                     /* QS:a4y6p */
    PLAYC = 3,                                                   /* QS:a5f2n */
    GRIDO = 4,                                                   /* QS:a6m7k */
    GRIDF = 5,                                                   /* QS:a7t2i */
    ENDINP = 6                                                   /* QS:a8a8f */
};                                                                    /* QS: */
                                                                      /* QS: */
struct Grid {                                                         /* QS: */
    int height;                                                       /* QS: */
    int width;                                                        /* QS: */
    int players;                                                      /* QS: */
    int turn;                                                         /* QS: */
    int emptycells;                                                   /* QS: */
    char* data;                                                       /* QS: */
};                                                                    /* QS: */
                                                                      /* QS: */
typedef struct Grid Grid;                                             /* QS: */
                                                                      /* QS: */
// Explanation of how the grid is stored in this code.                /* QS: */
// The cells used are:                                                /* QS: */
                                                                      /* QS: */
// N.N.N..                                                            /* QS: */
// *O*O*O*                                                            /* QS: */
// S.S.S..                                                            /* QS: */
                                                                      /* QS: */
// Where N indicates a north edge                                     /* QS: */
// * indicates a W/E edge                                             /* QS: */
// S indicates a south edge                                           /* QS: */
// O indicates owner information                                      /* QS: */
// . is an unused array location                                      /* QS: */
                                                                      /* QS: */
// Note that just as East of one cell is West of its neighbour,       /* QS: */
// so South edges become                                              /* QS: */
// The north edges of the cell below.                                 /* QS: */
                                                                      /* QS: */
// For example: in a 6 wide, 2 high grid, the following locations     /* QS: */
// in the array would be used.                                        /* QS: */
// North at [0], East at [15], South at [26], West at [13] and        /* QS: */
// the ownership information at [14]                                  /* QS: */
//                                                                    /* QS: */
                                                                      /* QS: */
                                                                      /* QS: */
/* Allocate a grid with the required parameters*/                     /* QS: */
Grid* alloc_grid(int height, int width, int players) {           /* QS:a9h3d */
    int i;                                                            /* QS: */
    Grid* res = malloc(sizeof(Grid));                            /* QS:b0o8a */
        /* This is more space than we need */                         /* QS: */
    res->data=malloc(sizeof(char) *                                   /* QS: */
        (1 + height*2) * (width*2 + 1));                         /* QS:b0v3y */
    for (i = 0; i < (1 + height*2) * (width*2 + 1); ++i) {       /* QS:b1c9v */
        res->data[i] = 0;                                        /* QS:b2j4t */
    }                                                                 /* QS: */
    res->players = players;                                      /* QS:b3q9q */
    res->height = height;                                        /* QS:b4x4o */
    res->width = width;                                          /* QS:b5e0m */
    res->turn = 0;                                               /* QS:b6l5j */
    res->emptycells = height * width;                            /* QS:b7s0h */
    return res;                                                       /* QS: */
}                                                                     /* QS: */
                                                                      /* QS: */
/* clean up and deallocate grid */                                    /* QS: */
void free_grid(Grid* g) {                                             /* QS: */
    free(g->data);                                                    /* QS: */
    free(g);                                                          /* QS: */
}                                                                     /* QS: */
                                                                      /* QS: */
/* North is side=0, East=1, ... */                                    /* QS: */
/* returns a pointer to the edge on _side_ of the cell row, col */    /* QS: */
char* get_edge(Grid* g, int row, int column, char side) {        /* QS:b8z5e */
    int index;                                                        /* QS: */
    switch (side) {                                                   /* QS: */
        case 0:                                                       /* QS: */
            index = row * 2 * (g->width*2 + 1) + column*2;       /* QS:b9g1c */
            break;                                                    /* QS: */
        case 1:                                                       /* QS: */
            index = (row * 2 + 1) *                                   /* QS: */
                 (g->width*2 + 1) + column*2 + 2;                /* QS:b9n6z */
            break;                                                    /* QS: */
        case 2:                                                       /* QS: */
            index = (row + 1) * 2 *                                   /* QS: */
                (g->width*2 + 1) + column*2;                     /* QS:c0u1x */
            break;                                                    /* QS: */
        default:        /* West */                                    /* QS: */
            index = (row * 2 + 1) *                                   /* QS: */
                (g->width*2 + 1) + column*2;                     /* QS:c1b7u */
    }                                                                 /* QS: */
    return g->data + index;                                           /* QS: */
}                                                                     /* QS: */
                                                                      /* QS: */
/* returns a pointer to the owner info for the specified cell */      /* QS: */
char* get_owner(Grid* g, int row, int column) {                  /* QS:c2i2s */
    int index = (row * 2 + 1) * (g->width*2 + 1) + column*2 + 1; /* QS:c3p7p */
    return g->data + index;                                           /* QS: */
}

/* Output grid to stdout */                                           /* QS: */
void print_grid(Grid* g) {                                            /* QS: */
    int r, c;                                                         /* QS: */
    for (r = 0; r < g->height; ++r) {                            /* QS:c4w2n */
            /* top edge */                                            /* QS: */
        printf(".");                                                  /* QS: */
        for (c = 0; c < g->width; ++c) {                         /* QS:c5d8k */
            printf("%c.", *get_edge(g, r, c, 0) ? '-' : ' ');         /* QS: */
        }                                                             /* QS: */
        printf("\n");                                                 /* QS: */
            /* East and West edges */                                 /* QS: */
        for (c = 0; c < g->width; ++c) {                         /* QS:c6k3i */
            char own = *get_owner(g, r, c);                           /* QS: */
            printf("%c%c", *get_edge(g, r, c, 3) ? '|' : ' ',         /* QS: */
                    own ? ('A' - 1) + own : ' ');                     /* QS: */
        }                                                             /* QS: */
            /* Now the East edge of the last cell */                  /* QS: */
        printf("%c\n", *get_edge(g, r, c - 1, 1) ? '|' : ' ');        /* QS: */
                                                                      /* QS: */
                                                                      /* QS: */
    }                                                                 /* QS: */
        /* Now the bottom edge */                                     /* QS: */
    printf(".");                                                      /* QS: */
    for (c = 0; c < g->width; ++c) {                             /* QS:c7r8f */
        printf("%c.",                                                 /* QS: */
           *get_edge(g, g->height - 1, c, 2) ? '-' : ' ');            /* QS: */
    }                                                                 /* QS: */
    printf("\n");                                                     /* QS: */
}                                                                     /* QS: */
                                                                      /* QS: */
/* for debug purposes */                                              /* QS: */
void dump_grid(Grid* g) {                                             /* QS: */
    int i;                                                            /* QS: */
    for (i = 0; i < sizeof(char) * (1 + g->height*2)                  /* QS: */
            * (g->width*2 + 1);                                  /* QS:c8y3d */
            ++i) {                                                    /* QS: */
        printf("%c", g->data[i] == 0 ? '0' : '1');                    /* QS: */
    }                                                                 /* QS: */
    printf("\n");                                                     /* QS: */
}


/* Ouput error message corrsponding to _e_ and then return _e_ */     /* QS: */
int error_msg(enum ERRS e) {                                          /* QS: */
    const char* m = 0;                                                /* QS: */
    switch (e) {                                                      /* QS: */
        case OK:                                                 /* QS:c9f9a */
            return OK;                                                /* QS: */
        case USAGE:                                              /* QS:c9m4y */
            m = "Usage: boxes height width playercount [filename]";   /* QS: */
            break;                                                    /* QS: */
        case DIM:                                                /* QS:d0t9v */
            m = "Invalid grid dimensions";                            /* QS: */
            break;                                                    /* QS: */
        case PLAYC:                                              /* QS:d1a5t */
            m = "Invalid player count";                               /* QS: */
            break;                                                    /* QS: */
        case GRIDO:                                              /* QS:d2h0r */
            m = "Invalid grid file";                                  /* QS: */
            break;                                                    /* QS: */
        case GRIDF:                                              /* QS:d3o5o */
            m = "Error reading grid contents";                        /* QS: */
            break;                                                    /* QS: */
        case ENDINP:                                             /* QS:d4v0m */
            m = "End of user input";                                  /* QS: */
            break;                                                    /* QS: */
        default:                                                      /* QS: */
            m = "??????";                                             /* QS: */
            e = 1;                                                    /* QS: */
    }                                                                 /* QS: */
    fprintf(stderr, "%s\n", m);                                  /* QS:d5c6j */
    return e;   /* to avoid calling exit() */                         /* QS: */
}                                                                     /* QS: */
 

/* helper function for load_grid. Reads edge information from _f_ */  /* QS: */
enum ERRS load_grid_edges(Grid* g, FILE* f) {                         /* QS: */
    int r, c;                                                         /* QS: */
    for (r = 0; r < g->height; ++r) {                                 /* QS: */
        int ed;                                                       /* QS: */
        /* top edge */                                                /* QS: */
        for (c = 0; c < g->width; ++c) {                         /* QS:d6j1h */
            ed = fgetc(f);                                       /* QS:d7q6e */
            if ((ed != '0') && (ed != '1')) {                    /* QS:d8x1c */
                /* bad char in the edge description */                /* QS: */
                return GRIDF;                                         /* QS: */
            }                                                         /* QS: */
            *get_edge(g, r, c, 0) = (ed == '1') ? 1 : 0;              /* QS: */
        }                                                             /* QS: */
        if ((ed = fgetc(f)) != '\n') {  /* clear end of line */       /* QS: */
            return GRIDF;                                             /* QS: */
        }                                                             /* QS: */
        /* vertical edges */                                          /* QS: */
        for (c = 0; c < g->width; ++c) {                         /* QS:d8e7z */
            ed = fgetc(f);                                       /* QS:d9l2x */
            if ((ed != '0') && (ed != '1')) {                    /* QS:e0s7u */
                /* bad char in the edge description */                /* QS: */
                return GRIDF;                                         /* QS: */
            }                                                         /* QS: */
            *get_edge(g, r, c, 3) = (ed == '1') ? 1 : 0;              /* QS: */
        }                                                             /* QS: */
            /* right most edge */                                     /* QS: */
        ed = fgetc(f);                                           /* QS:e1z2s */
        if ((ed != '0') && (ed != '1')) {                        /* QS:e2g8p */
            /* bad char in the edge description */                    /* QS: */
            return GRIDF;                                             /* QS: */
        }                                                             /* QS: */
        *get_edge(g, r, c - 1, 1) = (ed == '1') ? 1 : 0;              /* QS: */
        if (fgetc(f) != '\n') { /* clear end of line */               /* QS: */
            return GRIDF;                                             /* QS: */
        }                                                             /* QS: */
    }                                                                 /* QS: */
        /* final row */                                               /* QS: */
    for (c = 0; c < g->width; ++c) {                             /* QS:e3n3n */
        int ed = fgetc(f);                                       /* QS:e4u8k */
        if ((ed != '0') && (ed != '1')) {                             /* QS: */
            /* bad char in the edge description */                    /* QS: */
            return GRIDF;                                             /* QS: */
        }                                                             /* QS: */
        *get_edge(g, r - 1, c, 2) = (ed == '1') ? 1 : 0;              /* QS: */
    }                                                                 /* QS: */
    if (fgetc(f) != '\n') {     /* clear end of line */               /* QS: */
        return GRIDF;                                                 /* QS: */
    }                                                                 /* QS: */
    return OK;                                                        /* QS: */
                                                                      /* QS: */
}

/* read grid information from file */                                 /* QS: */
enum ERRS load_grid(Grid* g, const char* fname) {                     /* QS: */
    int r, c;                                                         /* QS: */
    int turn;                                                         /* QS: */
    enum ERRS res;                                                    /* QS: */
    FILE* f = fopen(fname, "r");                                 /* QS:e5b4i */
    if (!f) {                                                    /* QS:e6i9f */
        return GRIDO;                                                 /* QS: */
    }                                                                 /* QS: */
    if ((fgetuint(f, &turn, 3) != '\n') ||                            /* QS: */
         turn > g->players || turn < 1) {                             /* QS: */
        return GRIDF;                                                 /* QS: */
    }                                                                 /* QS: */
    res = load_grid_edges(g, f);                                 /* QS:e7p4d */
    if (res != OK) {                                             /* QS:e8w9a */
        return res;                                                   /* QS: */
    }                                                                 /* QS: */
    g->turn = turn - 1;                                          /* QS:e8d5y */
                                                                      /* QS: */
    /* Now to read in the ownership information */                    /* QS: */
    for (r = 0; r < g->height; ++r) {                                 /* QS: */
        int v;                                                        /* QS: */
        for (c = 0; c < g->width; ++c) {                         /* QS:e9k0w */
            char expsep = ',';  /* separator we expect */             /* QS: */
            if (c == g->width - 1) {                                  /* QS: */
                expsep = '\n';                                        /* QS: */
            }                                                         /* QS: */
            if (fgetuint(f, &v, 3) != expsep || (v > g->players)) {   /* QS: */
                return GRIDF;                                    /* QS:f0r5t */
            }                                                         /* QS: */
            *get_owner(g, r, c) = v;                                  /* QS: */
            if (v != 0) {                                             /* QS: */
                g->emptycells--;                                      /* QS: */
            }                                                         /* QS: */
        }                                                             /* QS: */
    }                                                                 /* QS: */
    return OK;                                                        /* QS: */
}                                                                     /* QS: */
                                                                      /* QS: */
/* read and discard chars from _f_ until EOF or end of line appears *//* QS: */
void gobble_eoln(FILE* f) {                                           /* QS: */
    int c;                                                            /* QS: */
    while (c = fgetc(f), c != '\n' && c != EOF) {                     /* QS: */
    }                                                                 /* QS: */
}                                                                     /* QS: */


/* move on to next player*/                                           /* QS: */
void next_player(Grid* g) {                                           /* QS: */
    g->turn = (g->turn + 1) % g->players;                             /* QS: */
}                                                                     /* QS: */
                                                                      /* QS: */
/* Add a horizontal edge to the top of the specified cell */          /* QS: */
/* To add the very bottom edge, use r==height */                      /* QS: */
void add_h(Grid* g, int r, int c) {                              /* QS:f1y0r */
    char* e;                                                          /* QS: */
    char edge = 0;                                                    /* QS: */
    int actualr = r;                                             /* QS:f2f6o */
    if ((r < 0) || (r > g->height) || (c < 0) || (c >= g->width)) {   /* QS: */
        return;                                                       /* QS: */
    }                                                                 /* QS: */
    if (r == g->height) {       /* right on the bottom edge */        /* QS: */
        edge = 2;                                                     /* QS: */
        r--;                                                     /* QS:f3m1m */
    }                                                                 /* QS: */
    e = get_edge(g, r, c, edge);                                      /* QS: */
    if (*e) {                                                         /* QS: */
        return;         /* the edge is already taken */               /* QS: */
    } else {                                                          /* QS: */
        BOOL found = FALSE;                                           /* QS: */
        *e = 1;                /* mark edge as taken */          /* QS:f4t6j */
        if (*get_edge(g, r, c, 0) && *get_edge(g, r, c, 1) &&         /* QS: */
                *get_edge(g, r, c, 2) &&                              /* QS: */
                *get_edge(g, r, c, 3)) {                              /* QS: */
                      /* all sides enclosed */                        /* QS: */
            *get_owner(g, r, c) = g->turn + 1;                        /* QS: */
            g->emptycells--;                                          /* QS: */
            found = TRUE;                                             /* QS: */
        }                                                             /* QS: */
          /* Now we need to consider the other possible cell */       /* QS: */
        if ((actualr == r) && (edge != 1) && (r != 0) &&              /* QS: */
                      /* Not at north or south extreme */             /* QS: */
                *get_edge(g, r - 1, c, 0) &&                          /* QS: */
                *get_edge(g, r - 1, c, 1) &&                          /* QS: */
                *get_edge(g, r - 1, c, 2) &&                          /* QS: */
                *get_edge(g, r - 1, c, 3)) {                          /* QS: */
            *get_owner(g, r - 1, c) = g->turn + 1;                    /* QS: */
            g->emptycells--;                                     /* QS:f5a2h */
            found = TRUE;                                        /* QS:f6h7e */
        }                                                             /* QS: */
        if (!found) {                                                 /* QS: */
            next_player(g);                                           /* QS: */
        }                                                             /* QS: */
        print_grid(g);                                                /* QS: */
    }                                                                 /* QS: */
}                                                                     /* QS: */


/* Add a vertical edge to the left of the specified cell */           /* QS: */
/* To add right most edge, use c==width */                            /* QS: */
void add_v(Grid* g, int r, int c) {                              /* QS:f7o2c */
    char* e;                                                          /* QS: */
    char edge = 3;                                                    /* QS: */
    int actualc = c;                                             /* QS:f7v7z */
    if ((r < 0) || (r >= g->height) || (c < 0) || (c > g->width)) {   /* QS: */
        return;                                                       /* QS: */
    }                                                                 /* QS: */
    if (r == g->height) {       /* right on the right edge */         /* QS: */
        edge = 1;                                                     /* QS: */
        c--;                                                          /* QS: */
    }                                                                 /* QS: */
    e = get_edge(g, r, c, edge);                                      /* QS: */
    if (*e) {                                                    /* QS:f8c3x */
        return;         /* the edge is already taken */               /* QS: */
    } else {                                                          /* QS: */
        BOOL found = FALSE;                                           /* QS: */
        *e = 1;         /* mark edge as taken */                 /* QS:f9j8u */
        if (*get_edge(g, r, c, 0) && *get_edge(g, r, c, 1) &&         /* QS: */
                *get_edge(g, r, c, 2) &&                              /* QS: */
                *get_edge(g, r, c, 3)) {                              /* QS: */
                  /* all sides enclosed */                            /* QS: */
            *get_owner(g, r, c) = g->turn + 1;                        /* QS: */
            g->emptycells--;                                     /* QS:g0q3s */
            found = TRUE;                                        /* QS:g1x8p */
        }                                                             /* QS: */
          /* Now we need to consider the other possible cell */       /* QS: */
        if ((actualc == c) && (edge != 1) && (c != 0) &&              /* QS: */
                /* Not at E or W extreme*/                            /* QS: */
                *get_edge(g, r, c - 1, 0) &&                          /* QS: */
                *get_edge(g, r, c - 1, 1) &&                          /* QS: */
                *get_edge(g, r, c - 1, 2) &&                          /* QS: */
                *get_edge(g, r, c - 1, 3)) {                          /* QS: */
            *get_owner(g, r, c - 1) = g->turn + 1;               /* QS:g2e4n */
            g->emptycells--;                                     /* QS:g3l9k */
            found = TRUE;                                        /* QS:g4s4i */
        }                                                             /* QS: */
        if (!found) {                                                 /* QS: */
            next_player(g);                                           /* QS: */
        }                                                             /* QS: */
        print_grid(g);                                                /* QS: */
    }                                                                 /* QS: */
}

/* Write grid information to _f_ */                                   /* QS: */
void write_grid(Grid* g, FILE* f) {                                   /* QS: */
    int r, c;                                                         /* QS: */
    fprintf(f, "%d\n", g->turn + 1);                             /* QS:g5z9f */
    for (r = 0; r < g->height; ++r) {                            /* QS:g6g5d */
        for (c = 0; c < g->width; ++c) {                         /* QS:g7n0b */
            if (*get_edge(g, r, c, 0) != 0) {                         /* QS: */
                fprintf(f, "1");                                 /* QS:g7u5y */
            } else {                                                  /* QS: */
                fprintf(f, "0");                                 /* QS:g8b1w */
            }                                                         /* QS: */
        }                                                             /* QS: */
        fprintf(f, "\n");                                        /* QS:g9i6t */
        for (c = 0; c < g->width; ++c) {                         /* QS:h0p1r */
            if (*get_edge(g, r, c, 3) != 0) {                         /* QS: */
                fprintf(f, "I");                                 /* QS:h1w6o */
            } else {                                                  /* QS: */
                fprintf(f, "0");                                 /* QS:h2d2m */
            }                                                         /* QS: */
        }                                                             /* QS: */
        if (*get_edge(g, r, c - 1, 1) != 0) {                         /* QS: */
            fprintf(f, "1");                                     /* QS:h3k7j */
        } else {                                                      /* QS: */
            fprintf(f, "0");                                     /* QS:h4r2h */
        }                                                             /* QS: */
        fprintf(f, "\n");                                        /* QS:h5y7e */
    }                                                                 /* QS: */
    /* Now the bottom row */                                          /* QS: */
    for (c = 0; c < g->width; ++c) {                             /* QS:h6f3c */
        if (*get_edge(g, r - 1, c, 2) != 0) {                         /* QS: */
            fprintf(f, "1");                                     /* QS:h6m8z */
        } else {                                                      /* QS: */
            fprintf(f, "0");                                     /* QS:h7t3x */
        }                                                             /* QS: */
    }                                                                 /* QS: */
    fprintf(f, "\n");                                                 /* QS: */
    for (r = 1; r < g->height; ++r) {                            /* QS:h8a9u */
        for (c = 0; c < g->width; ++c) {                         /* QS:h9h4s */
            char o = *get_owner(g, r, c);                             /* QS: */
            if (c > 0) {                                              /* QS: */
                fputc(',', f);                                   /* QS:i0o9p */
            }                                                         /* QS: */
                                                                      /* QS: */
            fprintf(f, "%d", (int)o);                                 /* QS: */
        }                                                             /* QS: */
        fprintf(f, "\n");                                             /* QS: */
    }                                                                 /* QS: */
}
/* Process the w action */                                            /* QS: */
void save_grid(Grid* g) {                                             /* QS: */
      /* The first step here is to find the name of the*/             /* QS: */
      /* file we should write to */                                   /* QS: */
    char* name;                                                       /* QS: */
    FILE* f = 0;                                                      /* QS: */
    name = get_line(stdin, 0);   /* No limit */                       /* QS: */
    f = fopen(name, "w");                                             /* QS: */
    free(name);                                                  /* QS:i1v4n */
    if (f == 0) {                                                     /* QS: */
        fprintf(stderr, "Can not open file for write\n");             /* QS: */
        return;                                                       /* QS: */
    }                                                                 /* QS: */
    write_grid(g, f);                                                 /* QS: */
    fclose(f);                                                   /* QS:i2c0l */
    fprintf(stderr, "Save complete\n");                          /* QS:i3j5i */
}


/* Read a move, process it, repeat until game over or error */        /* QS: */
enum ERRS game_loop(Grid* g) {                                        /* QS: */
    int ch;                                                           /* QS: */
    while (1) {                                                       /* QS: */
        if (g->emptycells == 0) {                                /* QS:i4q0g */
            return OK;                                                /* QS: */
        }                                                             /* QS: */
        printf("%c> ", g->turn + 'A');                           /* QS:i5x5d */
        ch = fgetc(stdin);                                       /* QS:i6e1b */
        if (ch == EOF) {                                              /* QS: */
            return ENDINP;                                            /* QS: */
        }                                                             /* QS: */
        if (ch == 'w') {                                              /* QS: */
            save_grid(g);                                             /* QS: */
        } else {        /* try to read a move */                      /* QS: */
            int r, c;                                                 /* QS: */
            ungetc(ch, stdin);                                   /* QS:i6l6y */
            ch = fgetuint(stdin, &r, 3);                         /* QS:i7s1w */
            if (ch == '\n') {                                         /* QS: */
                continue;      /* jump to next line */           /* QS:i8z6t */
            }                                                         /* QS: */
            if (ch != ' ') {                                          /* QS: */
                gobble_eoln(stdin); /* discard the rest of the line *//* QS: */
                continue;                                             /* QS: */
            }                                                         /* QS: */
            ch = fgetuint(stdin, &c, 3);                         /* QS:i9g2r */
                /* Now for a column */                                /* QS: */
            if (ch == '\n') {                                         /* QS: */
                continue;      /* jump to next line */                /* QS: */
            }                                                         /* QS: */
            if (ch != ' ') {                                     /* QS:j0n7o */
                gobble_eoln(stdin); /* discard the rest of the line *//* QS: */
                continue;                                             /* QS: */
            }                                                         /* QS: */
              /* So we have an integer for column */                  /* QS: */
            ch = fgetc(stdin);                                   /* QS:j1u2m */
            if ((ch != EOF) && (ch != '\n')) {                        /* QS: */
                gobble_eoln(stdin);  /* At this point, we are */      /* QS: */
            }       /* waiting at the beginning of the next line,*/   /* QS: */
            switch (ch) {                                             /* QS: */
                case 'h':                                             /* QS: */
                    add_h(g, r, c);                                   /* QS: */
                    break;                                            /* QS: */
                case 'v':                                             /* QS: */
                    add_v(g, r, c);                                   /* QS: */
                    break;                                            /* QS: */
                default:                                              /* QS: */
                    continue;                                         /* QS: */
            }                                                         /* QS: */
        }                                                             /* QS: */
    }                                                                 /* QS: */
    return OK;                                                        /* QS: */
}

/* Determine and print winners */                                     /* QS: */
void print_winners(Grid* g) {                                         /* QS: */
      /* since we have no totals, we need to compute them now */      /* QS: */
    int* counts = malloc(sizeof(int) * g->players);                   /* QS: */
    int r, c;                                                         /* QS: */
    int max = 0;                                                      /* QS: */
    BOOL first = TRUE;                                           /* QS:j2b8j */
    memset(counts, 0, sizeof(int) * g->players);                      /* QS: */
    for (r = 0; r < g->height; ++r) {                            /* QS:j3i3h */
        for (c = 0; c < g->width; ++c) {                         /* QS:j4p8e */
            char o = *get_owner(g, r, c);                             /* QS: */
            if (o != 0) {                                             /* QS: */
                counts[o - 1]++;                                      /* QS: */
            }                                                         /* QS: */
        }                                                             /* QS: */
                                                                      /* QS: */
    }                                                                 /* QS: */
      /* find the highest value */                                    /* QS: */
    for (r = 0; r < g->players; ++r) {                           /* QS:j5w3c */
        if (counts[r] > max) {                                   /* QS:j5d9z */
            max = counts[r];                                     /* QS:j6k4x */
        }                                                             /* QS: */
    }                                                                 /* QS: */
    printf("Winner(s):");                                        /* QS:j7r9u */
    for (r = 0; r < g->players; ++r) {                           /* QS:j8y4s */
        if (counts[r] == max) {                                  /* QS:j9f0q */
            if (first) {                                         /* QS:k0m5n */
                first = FALSE;                                        /* QS: */
            } else {                                                  /* QS: */
                printf(",");                                     /* QS:k1t0l */
            }                                                         /* QS: */
            printf(" %c", 'A' + r);                              /* QS:k2a6i */
        }                                                             /* QS: */
    }                                                                 /* QS: */
    printf("\n");                                                     /* QS: */
    free(counts);                                                     /* QS: */
}                                                                     /* QS: */

int main(int argc, char** argv) {                                     /* QS: */
    char* errc1, *errc2;                                              /* QS: */
    int h, w, players;                                                /* QS: */
    enum ERRS err;                                                    /* QS: */
    Grid* g = 0;                                                 /* QS:k3h1g */
    if ((argc < 4) || (argc > 5)) {                              /* QS:k4o6d */
        return error_msg(USAGE);                                      /* QS: */
    }                                                                 /* QS: */
    h = strtol(argv[2], &errc1, 10);                             /* QS:k5v1b */
    w = strtol(argv[2], &errc2, 10);                             /* QS:k5c7y */
    if ((h < 2) || (w < 2) || (h > 999) || (w > 999)                  /* QS: */
            || (*errc1 != '\0') || (*errc2 != '\0')) {                /* QS: */
        return error_msg(DIM);                                        /* QS: */
    }                                                                 /* QS: */
    players = strtol(argv[3], &errc1, 10);                            /* QS: */
    if ((players < 2) || (players > 100) || (*errc1 != '\0')) {       /* QS: */
        return error_msg(PLAYC);                                 /* QS:k6j2w */
    }                                                                 /* QS: */
    g = alloc_grid(h, w, players);                                    /* QS: */
                                                                      /* QS: */
    if (argc == 5) {                                                  /* QS: */
        if ((err = load_grid(g, argv[4])) != OK) {               /* QS:k7q7t */
            free_grid(g);                                        /* QS:k8x2r */
            return error_msg(err);                                    /* QS: */
        }                                                             /* QS: */
        if (g->emptycells == 0) {                                     /* QS: */
            free_grid(g);                                        /* QS:k9e8o */
            return error_msg(GRIDF);                                  /* QS: */
                /* If the game is complete already, error time*/      /* QS: */
        }                                                             /* QS: */
    }                                                                 /* QS: */
    print_grid(g);                                                    /* QS: */
    err = game_loop(g);                                          /* QS:l0l3m */
    if (err == OK) {                                             /* QS:l1s8j */
        print_winners(g);                                             /* QS: */
    }                                                                 /* QS: */
    free_grid(g);                                                     /* QS: */
    return error_msg(err);                                            /* QS: */
}                                                                     /* QS: */
              

