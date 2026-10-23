// MK7 Bot Manager - a 3DS homebrew app (.3dsx / CIA). Build with devkitPro + libctru.
// INJECT: renames MK7Bot.3gx.off -> MK7Bot.3gx in every /luma/plugins/<titleID>/ folder.
// EJECT : renames it back, so Luma no longer loads it. Takes effect at the next game launch.
// The plugin lives only in the game's memory, so closing the game always removes it too.
#include <3ds.h>
#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#define BASE "sdmc:/luma/plugins"
#define ON   "MK7Bot.3gx"
#define OFF  "MK7Bot.3gx.off"

static int exists(const char *p) { struct stat s; return stat(p, &s) == 0; }

// returns number of folders touched; counts how many are currently on/off
static int scan(int action, int *on, int *off)
{
    DIR *d = opendir(BASE);
    if (!d) return -1;
    struct dirent *e;
    int touched = 0;
    *on = *off = 0;
    while ((e = readdir(d))) {
        if (e->d_name[0] == '.') continue;
        char dir[300], pOn[400], pOff[400];
        snprintf(dir, sizeof dir, BASE "/%s", e->d_name);
        snprintf(pOn, sizeof pOn, "%s/" ON, dir);
        snprintf(pOff, sizeof pOff, "%s/" OFF, dir);
        if (action == 1 && exists(pOff) && !exists(pOn)) { if (rename(pOff, pOn) == 0) touched++; }
        if (action == 2 && exists(pOn)) { if (rename(pOn, pOff) == 0) touched++; }
        if (exists(pOn)) (*on)++;
        if (exists(pOff)) (*off)++;
    }
    closedir(d);
    return touched;
}

static void wipe(void)
{
    DIR *d = opendir("sdmc:/MK7Bot");
    if (!d) return;
    struct dirent *e;
    while ((e = readdir(d))) {
        size_t n = strlen(e->d_name);
        if (n > 4 && strcmp(e->d_name + n - 4, ".bin") == 0) {
            char p[300];
            snprintf(p, sizeof p, "sdmc:/MK7Bot/%s", e->d_name);
            remove(p);
        }
    }
    closedir(d);
}

static void draw(const char *msg)
{
    int on, off;
    int r = scan(0, &on, &off);
    consoleClear();
    printf("\x1b[1;1H=== MK7 Bot Manager ===\n\n");
    if (r < 0) printf("No /luma/plugins folder found.\nInstall Luma3DS first.\n\n");
    else printf("Bot status: %s\n(folders: %d injected, %d ejected)\n\n",
                on ? "INJECTED (loads next game launch)" : (off ? "EJECTED (normal game)" : "NOT INSTALLED"), on, off);
    printf("A  Inject bot\nB  Eject bot (back to normal)\nX  Forget all learned tracks\nSTART  Exit\n\n");
    printf("Then start Mario Kart 7 from HOME.\nClosing the game also removes the bot.\n\n%s\n", msg);
}

int main(void)
{
    gfxInitDefault();
    consoleInit(GFX_TOP, NULL);
    const char *msg = "";
    draw(msg);
    while (aptMainLoop()) {
        hidScanInput();
        u32 k = hidKeysDown();
        if (k & KEY_START) break;
        int on, off;
        if (k & KEY_A) { int n = scan(1, &on, &off); msg = n > 0 ? "Injected." : "Nothing to inject. Put MK7Bot.3gx (or .3gx.off)\nin /luma/plugins/<titleID>/ first."; draw(msg); }
        if (k & KEY_B) { int n = scan(2, &on, &off); msg = n > 0 ? "Ejected." : "Nothing to eject."; draw(msg); }
        if (k & KEY_X) { wipe(); msg = "Learned tracks erased."; draw(msg); }
        gfxFlushBuffers(); gfxSwapBuffers(); gspWaitForVBlank();
    }
    gfxExit();
    return 0;
}
